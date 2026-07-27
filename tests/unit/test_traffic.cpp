#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/traffic/queue_model.hpp>
#include <nomad/traffic/ltm_model.hpp>

using namespace nomad;

// ── Single-edge graph ─────────────────────────────────────────────────────────
static Graph make_single_edge(float length_m = 1000.0f,
                                float speed_ms = 10.0f,
                                float capacity = 1600.0f) {
    Graph g;
    g.nodes.resize(2);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.row_ptr = {0, 1, 2};
    g.col_idx = {0, 1};
    g.edges.resize(2);

    EdgeData fwd{};
    fwd.target = 1; fwd.length_m = length_m;
    fwd.free_flow_speed = speed_ms; fwd.capacity = capacity;
    fwd.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[0] = fwd;

    EdgeData rev = fwd;
    rev.target = 0;
    g.edges[1] = rev;

    g.geom_ptr.assign(3, 0);
    return g;
}

TEST_CASE("QueueModel: on_enter returns free-flow time when empty", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f); // ff_time = 100s
    QueueTrafficModel model(g);
    SimTime exit_time = model.on_enter(0, 0, 0.0);
    REQUIRE_THAT(static_cast<float>(exit_time),
                  Catch::Matchers::WithinAbs(100.0f, 1.0f));
}

TEST_CASE("QueueModel: on_exit succeeds when no spillback", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f);
    QueueTrafficModel model(g);
    model.on_enter(0, 0, 0.0);
    bool ok = model.on_exit(0, kInvalidEdge, 0, 100.0);
    REQUIRE(ok);
}

TEST_CASE("QueueModel: travel time increases with congestion", "[traffic]") {
    auto g = make_single_edge(500.0f, 10.0f, 800.0f); // small cap
    QueueTrafficModel model(g);

    SimTime t0 = model.on_enter(0, 0, 0.0);    // first agent, should be ff
    float ff = 500.0f / 10.0f; // 50s

    // Fill the link close to capacity to see BPR effect
    for (int i = 1; i < 10; ++i) model.on_enter(0, i, 0.0);
    SimTime t1 = model.on_enter(0, 10, 0.0);   // congested entry

    // Congested time should be ≥ free-flow time
    REQUIRE(static_cast<float>(t1) >= static_cast<float>(t0));
}

TEST_CASE("QueueModel: current_travel_time is positive", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.current_travel_time(0) > 0.0f);
    REQUIRE(model.current_travel_time(1) > 0.0f);
}

TEST_CASE("QueueModel: link_states span has correct size", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.link_states().size() == g.num_edges());
}

TEST_CASE("QueueModel: model_name is 'queue'", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.model_name() == "queue");
}

TEST_CASE("QueueModel: occupancy updates after on_enter", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    float before = model.link_states()[0].occupancy.load();
    model.on_enter(0, 0, 0.0);
    float after  = model.link_states()[0].occupancy.load();
    REQUIRE(after > before);
}

TEST_CASE("QueueModel: update does not crash", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    model.on_enter(0, 0, 0.0);
    REQUIRE_NOTHROW(model.update(100.0));
}

// ── Three-node chain: edge 0 (0→1), edge 1 (1→2) ──────────────────────────────
static Graph make_chain(float len0 = 500.0f, float len1 = 50.0f,
                          float speed_ms = 10.0f,
                          float cap0 = 1600.0f, float cap1 = 100.0f) {
    Graph g;
    g.nodes.resize(3);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};
    g.row_ptr = {0, 1, 2, 2};
    g.col_idx = {0, 1};
    g.edges.resize(2);

    EdgeData e0{};
    e0.target = 1; e0.length_m = len0;
    e0.free_flow_speed = speed_ms; e0.capacity = cap0;
    e0.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[0] = e0;

    EdgeData e1{};
    e1.target = 2; e1.length_m = len1;
    e1.free_flow_speed = speed_ms; e1.capacity = cap1;
    e1.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[1] = e1;

    g.geom_ptr.assign(3, 0);
    return g;
}

TEST_CASE("LtmModel: model_name is 'ltm'", "[traffic]") {
    auto g = make_single_edge();
    LtmTrafficModel model(g);
    REQUIRE(model.model_name() == "ltm");
}

TEST_CASE("LtmModel: on_exit blocked when downstream link is full (spillback)", "[traffic]") {
    // edge1 has capacity=100 veh/h → 1 lane, length=50m (the effective-length
    // floor) → storage_veh = 50 * 1 * (1/7.5) ≈ 6.67 vehicles.
    auto g = make_chain();
    LtmTrafficModel model(g);

    // Fill edge1 to its storage capacity.
    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);

    // An agent on edge0 trying to move onto the saturated edge1 is blocked.
    model.on_enter(0, 100, 0.0);
    bool ok = model.on_exit(0, /*next=*/1, 100, 100.0);
    REQUIRE_FALSE(ok);

    // The blocked agent's own link occupancy is unchanged (still on edge0).
    REQUIRE(model.link_states()[0].occupancy.load() == 1.0f);
}

TEST_CASE("LtmModel: on_exit succeeds once downstream link has spare capacity", "[traffic]") {
    auto g = make_chain();
    LtmTrafficModel model(g);

    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);
    model.on_enter(0, 100, 0.0);
    REQUIRE_FALSE(model.on_exit(0, 1, 100, 100.0));

    // Downstream link drains.
    REQUIRE(model.on_exit(1, kInvalidEdge, 0, 100.0));

    // Now there is spare storage on edge1 — the agent on edge0 can proceed.
    REQUIRE(model.on_exit(0, 1, 100, 101.0));
}

TEST_CASE("LtmModel: next == kInvalidEdge (trip end) always allows exit", "[traffic]") {
    auto g = make_chain();
    LtmTrafficModel model(g);
    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);
    // Even though edge1 (unrelated to this exit) is saturated, exiting to
    // kInvalidEdge never checks a downstream link.
    REQUIRE(model.on_exit(1, kInvalidEdge, 0, 100.0));
}

TEST_CASE("LtmModel: has_capacity false once edge is saturated", "[traffic]") {
    // edge1 storage_veh ≈ 6.67 (see spillback test above) — 7 agents saturate it.
    auto g = make_chain();
    LtmTrafficModel model(g);
    REQUIRE(model.has_capacity(1));
    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);
    REQUIRE_FALSE(model.has_capacity(1));
}

TEST_CASE("LtmModel: has_capacity true again after downstream link drains", "[traffic]") {
    auto g = make_chain();
    LtmTrafficModel model(g);
    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);
    REQUIRE_FALSE(model.has_capacity(1));
    model.on_exit(1, kInvalidEdge, 0, 100.0);
    REQUIRE(model.has_capacity(1));
}

TEST_CASE("QueueModel: has_capacity always true (no storage/spillback concept)", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    for (AgentId a = 0; a < 50; ++a) model.on_enter(0, a, 0.0);
    REQUIRE(model.has_capacity(0));  // default no-op from ITrafficModel
}

// ── Discharge-rate token bucket (opt-in) ──────────────────────────────────────
static LtmTrafficModel::Config discharge_cfg(float burst_s = 3.0f) {
    LtmTrafficModel::Config cfg;
    cfg.enable_discharge_cap = true;
    cfg.discharge_burst_s    = burst_s;
    return cfg;
}

TEST_CASE("LtmModel: discharge cap throttles a sustained burst below capacity_veh_s", "[traffic]") {
    // capacity=3600 veh/h -> capacity_veh_s = 1.0; burst_s=3 -> max_credit=3.0
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);
    LtmTrafficModel model(g, discharge_cfg(3.0f));
    for (AgentId a = 0; a < 15; ++a) model.on_enter(0, a, 0.0);

    int exits_at_t0 = 0;
    for (AgentId a = 0; a < 15; ++a)
        if (model.on_exit(0, kInvalidEdge, a, 0.0)) ++exits_at_t0;
    REQUIRE(exits_at_t0 == 3);  // only the burst buffer's worth clears instantly

    // 3s later, exactly one more max_credit's worth (3.0 * 1.0 veh/s) has
    // accrued -- still far below letting all remaining 12 through.
    int exits_at_t3 = 0;
    for (AgentId a = 0; a < 12; ++a)
        if (model.on_exit(0, kInvalidEdge, a, 3.0)) ++exits_at_t3;
    REQUIRE(exits_at_t3 == 3);
}

TEST_CASE("LtmModel: discharge cap never blocks a single isolated reasonable exit", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);
    LtmTrafficModel model(g, discharge_cfg(10.0f));
    model.on_enter(0, 0, 0.0);
    REQUIRE(model.on_exit(0, kInvalidEdge, 0, 0.0));    // starts full -> never blocked

    model.on_enter(0, 1, 100.0);
    REQUIRE(model.on_exit(0, kInvalidEdge, 1, 100.0));  // ample refill at low rate
}

TEST_CASE("LtmModel: discharge cap floors max_credit at 1 veh for very low capacity", "[traffic]") {
    // capacity=100 veh/h -> capacity_veh_s ~ 0.0278; burst_s=10 -> raw product
    // 0.278 < 1 -- without the max(1.0, ...) floor this would block forever.
    auto g = make_single_edge(1000.0f, 10.0f, 100.0f);
    LtmTrafficModel model(g, discharge_cfg(10.0f));
    model.on_enter(0, 0, 0.0);
    REQUIRE(model.on_exit(0, kInvalidEdge, 0, 0.0));
}

TEST_CASE("LtmModel: discharge cap allows a zero-elapsed-time burst (not a rigid headway)", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);  // capacity_veh_s = 1.0
    LtmTrafficModel model(g, discharge_cfg(3.0f));       // max_credit = 3.0
    for (AgentId a = 0; a < 4; ++a) model.on_enter(0, a, 0.0);

    // All three succeed at the SAME instant t=0 -- a rigid 1/capacity_veh_s=1s
    // headway could never allow this.
    REQUIRE(model.on_exit(0, kInvalidEdge, 0, 0.0));
    REQUIRE(model.on_exit(0, kInvalidEdge, 1, 0.0));
    REQUIRE(model.on_exit(0, kInvalidEdge, 2, 0.0));
    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 3, 0.0));  // buffer now empty
}

TEST_CASE("LtmModel: discharge cap disabled by default preserves current spillback-only behavior", "[traffic]") {
    auto g = make_chain();
    LtmTrafficModel model(g, LtmTrafficModel::Config{});  // enable_discharge_cap=false
    for (AgentId a = 0; a < 50; ++a) model.on_enter(0, a, 0.0);
    for (AgentId a = 0; a < 50; ++a)
        REQUIRE(model.on_exit(0, kInvalidEdge, a, 0.0));  // no rate limiting at all
}

// ── Discharge-gate wait signal (opt-in, Phase 4) ──────────────────────────────
// travel_time_for() is private; observed indirectly via current_travel_time(),
// which update() refreshes from travel_time_for() for any edge with
// occupancy > 0 (a blocked-but-not-exited agent keeps the edge in the active
// set, so update() always has something to recompute here).

TEST_CASE("LtmModel: gate wait grows monotonically while the discharge check keeps blocking", "[traffic]") {
    // capacity=3600 veh/h -> capacity_veh_s = 1.0; burst_s=1 -> max_credit=1.0,
    // so a second agent packed onto a nearly-empty bucket blocks immediately.
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);
    LtmTrafficModel model(g, discharge_cfg(1.0f));
    model.on_enter(0, 0, 0.0);
    model.on_enter(0, 1, 0.0);

    REQUIRE(model.on_exit(0, kInvalidEdge, 0, 0.0));         // consumes the only credit
    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 1, 0.0));   // blocked, gate wait starts at 0
    model.update(0.0);
    float tt0 = model.current_travel_time(0);

    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 1, 0.5));   // still blocked, wait=0.5s
    model.update(0.5);
    float tt1 = model.current_travel_time(0);

    // Stay within the 1.0s burst window (max_credit=1.0, refills at 1.0/s) --
    // past 1.0s since the block started the bucket would be full again and
    // this exit would succeed, which is a different scenario (tested below).
    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 1, 0.9));   // still blocked, wait=0.9s
    model.update(0.9);
    float tt2 = model.current_travel_time(0);

    REQUIRE(tt1 > tt0);
    REQUIRE(tt2 > tt1);
}

TEST_CASE("LtmModel: a successful exit resets gate wait immediately", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);  // capacity_veh_s = 1.0
    LtmTrafficModel model(g, discharge_cfg(1.0f));       // max_credit = 1.0
    model.on_enter(0, 0, 0.0);
    model.on_enter(0, 1, 0.0);

    REQUIRE(model.on_exit(0, kInvalidEdge, 0, 0.0));
    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 1, 0.1));   // blocked, gate wait starts at 0
    // Block persists a bit longer -- gate_wait_s is now nonzero (duration since 0.1).
    REQUIRE_FALSE(model.on_exit(0, kInvalidEdge, 1, 0.3));
    model.update(0.3);
    REQUIRE(model.current_travel_time(0) > 100.0f);          // ff=100s + gate_wait_s(~0.2s)

    // Let credit refill fully (well past the 1.0s burst window), exit succeeds.
    REQUIRE(model.on_exit(0, kInvalidEdge, 1, 5.0));
    model.update(5.0);
    // Only one agent left (occupancy=1, well under 80% of storage) -> plain
    // free-flow time, no residual gate term.
    REQUIRE(model.current_travel_time(0) == Catch::Approx(100.0f).epsilon(0.01));
}

TEST_CASE("LtmModel: gate wait term is additive only when discharge cap is enabled", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);

    // Cap disabled: the discharge-check branch in on_exit() never executes,
    // so gate_wait_s can never become nonzero -- current_travel_time() must
    // match the plain BPR-only formula exactly (byte-identical to pre-Phase-4).
    LtmTrafficModel off_model(g, LtmTrafficModel::Config{});
    for (AgentId a = 0; a < 30; ++a) off_model.on_enter(0, a, 0.0);  // occ=30 (storage_veh=300)
    off_model.update(0.0);
    float tt_off = off_model.current_travel_time(0);

    // Same occupancy, cap enabled but burst generous enough to never block --
    // should match the disabled case exactly (gate_wait_s stays 0).
    LtmTrafficModel on_model(g, discharge_cfg(1000.0f));
    for (AgentId a = 0; a < 30; ++a) on_model.on_enter(0, a, 0.0);
    on_model.update(0.0);
    float tt_on_unblocked = on_model.current_travel_time(0);

    REQUIRE(tt_off == Catch::Approx(tt_on_unblocked).epsilon(0.001));
}

TEST_CASE("LtmModel: occupancy-driven congestion alone never sets gate wait", "[traffic]") {
    // High occupancy (vc near/at 1) but discharge cap generous enough that
    // on_exit() never fails the credit check -- gate_wait_s must stay 0, i.e.
    // current_travel_time() must equal the plain BPR value for this vc.
    auto g = make_single_edge(1000.0f, 10.0f, 3600.0f);  // storage_veh = 300
    LtmTrafficModel model(g, discharge_cfg(1000.0f));  // effectively never binds
    for (AgentId a = 0; a < 270; ++a) model.on_enter(0, a, 0.0);  // vc = 270/300 = 0.9
    model.update(0.0);
    float tt = model.current_travel_time(0);

    for (AgentId a = 0; a < 5; ++a)
        REQUIRE(model.on_exit(0, kInvalidEdge, a, 0.0));  // ample credit, never blocked
    model.update(0.0);
    REQUIRE(model.current_travel_time(0) == Catch::Approx(tt).epsilon(0.05));
}

TEST_CASE("LtmModel: gate-wait fields leave disabled-by-default spillback behavior byte-identical", "[traffic]") {
    // Re-run the exact disabled-cap spillback scenario from earlier in this
    // file with the new Cell fields present -- guards against an accidental
    // unconditional write to gate_block_since/gate_wait_s.
    auto g = make_chain();
    LtmTrafficModel model(g, LtmTrafficModel::Config{});  // enable_discharge_cap=false
    for (AgentId a = 0; a < 50; ++a) model.on_enter(0, a, 0.0);
    for (AgentId a = 0; a < 50; ++a)
        REQUIRE(model.on_exit(0, kInvalidEdge, a, 0.0));  // no rate limiting at all
    REQUIRE(model.link_states()[0].occupancy.load() == 0.0f);
}

TEST_CASE("LtmModel: force_remove restores occupancy/capacity", "[traffic]") {
    auto g = make_chain();
    LtmTrafficModel model(g);
    for (AgentId a = 0; a < 7; ++a) model.on_enter(1, a, 0.0);
    REQUIRE(model.link_states()[1].occupancy.load() == 7.0f);

    model.force_remove(1, 0);
    REQUIRE(model.link_states()[1].occupancy.load() == 6.0f);

    // Freed-up capacity now unblocks a waiting exit onto edge1.
    model.on_enter(0, 100, 0.0);
    REQUIRE(model.on_exit(0, 1, 100, 100.0));
}
