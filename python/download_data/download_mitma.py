from pathlib import Path
from urllib.parse import urlparse, unquote
from datetime import datetime, date
import argparse
import calendar
import re
import time
import requests
import xml.etree.ElementTree as ET


MITMA_ROOT = "https://movilidad-opendata.mitma.es/"
MITMA_RSS = MITMA_ROOT + "RSS.xml"

TARGET_PREFIX = "estudios_basicos/por-distritos/viajes/ficheros-diarios/"

OUT_DIR = Path("/home/fede/code/nomad/data/od_raw")


def parse_date(s):
    return datetime.strptime(s, "%Y-%m-%d").date()


def month_to_date_range(month_str):
    """
    Converte 'YYYY-MM' in primo e ultimo giorno del mese.
    Esempio:
        '2022-11' -> 2022-11-01, 2022-11-30
    """
    try:
        year, month = map(int, month_str.split("-"))
    except ValueError:
        raise ValueError("Il mese deve essere nel formato YYYY-MM, esempio: 2022-11")

    if month < 1 or month > 12:
        raise ValueError("Il mese deve essere tra 01 e 12")

    last_day = calendar.monthrange(year, month)[1]

    start_date = date(year, month, 1)
    end_date = date(year, month, last_day)

    return start_date, end_date


def extract_date_from_url(url):
    name = Path(unquote(urlparse(url).path)).name
    m = re.search(r"(\d{8})", name)

    if not m:
        return None

    return datetime.strptime(m.group(1), "%Y%m%d").date()


def extract_urls_from_rss(xml_text):
    root = ET.fromstring(xml_text)
    urls = set()

    for elem in root.iter():
        candidates = list(elem.attrib.values())

        if elem.text:
            candidates.append(elem.text)

        for value in candidates:
            value = value.strip()

            if not value:
                continue

            if value.startswith("http"):
                urls.add(value)

            elif "estudios_basicos/" in value:
                urls.add(MITMA_ROOT + value.lstrip("/"))

    return sorted(urls)


def select_viajes_files(urls, start_date, end_date):
    selected = []

    for url in urls:
        low = url.lower()

        if TARGET_PREFIX.lower() not in low:
            continue

        if not low.endswith((".csv.gz", ".csv", ".zip")):
            continue

        d = extract_date_from_url(url)

        if d is None:
            continue

        if start_date <= d <= end_date:
            selected.append((d, url))

    return sorted(selected)


def output_path_for(url):
    path = unquote(urlparse(url).path)
    name = Path(path).name

    month_match = re.search(r"/(\d{4}-\d{2})/", path)
    month = month_match.group(1) if month_match else "unknown_month"

    return OUT_DIR / month / name


def download_file(session, url, out_path, retries=5):
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if out_path.exists() and out_path.stat().st_size > 0:
        print(f"SKIP già esiste: {out_path}")
        return

    tmp_path = out_path.with_suffix(out_path.suffix + ".part")

    for attempt in range(1, retries + 1):
        try:
            with session.get(url, stream=True, timeout=(15, 180)) as r:
                r.raise_for_status()

                content_type = r.headers.get("content-type", "").lower()

                if "text/html" in content_type:
                    raise RuntimeError(
                        f"MITMA ha restituito HTML invece del CSV: {url}"
                    )

                with open(tmp_path, "wb") as f:
                    for chunk in r.iter_content(chunk_size=1024 * 1024):
                        if chunk:
                            f.write(chunk)

            if tmp_path.stat().st_size == 0:
                raise RuntimeError("Download vuoto")

            tmp_path.replace(out_path)
            print(f"OK: {out_path}")
            return

        except Exception as e:
            print(f"Tentativo {attempt}/{retries} fallito per {url}: {e}")
            time.sleep(2 * attempt)

    raise RuntimeError(f"Download fallito definitivamente: {url}")


def select_zones_files(urls):
    """Trova nel RSS i file di zonificazione (shapefile zip o gpkg).
    Cerca URL che contengono 'zonif' nel path e hanno estensione .zip o .gpkg.
    """
    selected = []
    for url in urls:
        low = url.lower()
        # Deve contenere 'zonif' (zonificacion, zonificación, zonification…)
        if 'zonif' not in low:
            continue
        # Solo shapefile zip o gpkg (non csv.gz o txt.gz che sono dati OD)
        if not low.endswith((".zip", ".gpkg")):
            continue
        selected.append(url)
    return selected


ZONES_STATIC_URLS = [
    # Prova in ordine: URL noti del portale MITMA per la zonificazione distretti
    MITMA_ROOT + "estudios_basicos/por-distritos/zonificacion/zonificacion_distritos.zip",
    MITMA_ROOT + "zonificacion/zonificacion_distritos.zip",
    MITMA_ROOT + "estudios_basicos/por-distritos/zonificacion/Zonificacion_distritos.zip",
    MITMA_ROOT + "zonificacion/Zonificacion_por_distritos.zip",
    MITMA_ROOT + "zonificacion/ZonificacionDist_MITMA.zip",
]


def download_zones(session, rss_urls, out_dir):
    import zipfile

    # Prima prova il RSS
    zone_urls = select_zones_files(rss_urls)

    # Poi prova URL statici noti
    if not zone_urls:
        print("Non trovato nell'RSS, provo URL statici del portale...")
        for url in ZONES_STATIC_URLS:
            try:
                r = session.head(url, timeout=10)
                if r.status_code == 200:
                    zone_urls = [url]
                    print(f"  Trovato: {url}")
                    break
            except Exception:
                continue

    if not zone_urls:
        print(
            "File zone non trovato automaticamente.\n"
            "Scarica manualmente il pacchetto zonificazione dal portale MITMA:\n"
            "  https://movilidad-opendata.mitma.es\n"
            "ed estrailo in data/od_raw/ (deve contenere .shp + .dbf + .shx + .prj)."
        )
        return

    for url in zone_urls:
        name = Path(unquote(urlparse(url).path)).name
        out_path = out_dir / name
        print(f"Scarico: {name}")
        download_file(session, url, out_path)

        if name.endswith(".zip"):
            with zipfile.ZipFile(out_path) as zf:
                zf.extractall(out_dir)
            print(f"  estratto in {out_dir}")
            out_path.unlink()   # rimuovi lo zip dopo l'estrazione


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scarica automaticamente i dati MITMA viajes por distritos per un mese."
    )

    parser.add_argument(
        "--month",
        help="Mese da scaricare nel formato YYYY-MM, esempio: 2022-11",
    )
    parser.add_argument(
        "--zones",
        action="store_true",
        help="Scarica anche i file di zonificazione (shapefile distretti)",
    )
    parser.add_argument(
        "--out-dir",
        default=str(OUT_DIR),
        help=f"Cartella di output. Default: {OUT_DIR}",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if not args.month and not args.zones:
        print("Specifica --month YYYY-MM oppure --zones (o entrambi).")
        return

    global OUT_DIR
    OUT_DIR = Path(args.out_dir)

    session = requests.Session()
    session.headers.update({"User-Agent": "Mozilla/5.0"})

    print("Scarico RSS/XML MITMA...")
    r = session.get(MITMA_RSS, timeout=60)
    r.raise_for_status()
    urls = extract_urls_from_rss(r.text)
    print(f"Link totali trovati nell'RSS: {len(urls)}")

    if args.zones:
        print("\n── Zone ────────────────────────────────")
        download_zones(session, urls, OUT_DIR)

    if args.month:
        print(f"\n── Viajes {args.month} ─────────────────────────")
        start_date, end_date = month_to_date_range(args.month)
        print(f"Periodo: {start_date} → {end_date}")
        print(f"Output dir: {OUT_DIR}")

        files = select_viajes_files(urls, start_date, end_date)
        print(f"File trovati: {len(files)}")
        if not files:
            print("Nessun file trovato. Controlla il mese o TARGET_PREFIX.")
            return
        for d, url in files:
            download_file(session, url, output_path_for(url))

    print("\nDownload completato.")


if __name__ == "__main__":
    main()