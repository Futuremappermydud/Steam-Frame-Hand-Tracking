import gzip
import hashlib
import io
import json
import lzma
from pathlib import Path
import sys
import tarfile
import urllib.request

ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'build' / 'mercury' / 'deps'
ORT = 'https://raw.githubusercontent.com/microsoft/onnxruntime/v1.22.1/include/onnxruntime/core/session/'
# The ONNX Runtime headers are also stored in this repository; a copy that matches its pin is used without a download.
BUNDLED = Path(__file__).resolve().parent / 'third_party' / 'onnxruntime' / 'include'
PINS = {
    'https://deb.debian.org/debian/pool/main/e/eigen3/libeigen3-dev_3.4.0-4_all.deb':
        '1a167bb381acb90c1d8450a8f3e66a09d216dbc64b33ac8a082a0912513ef939',
    ORT + 'onnxruntime_c_api.h': 'd683537d0fdc29e977b5520f7f15d87a0ac212ae6d94fa9be8893a52655621ae',
    ORT + 'onnxruntime_cxx_api.h': '28b21dfc371fb53d1ba67356adeb124f36643ee5ab6f1ad2e9afbf09eb4bf305',
    ORT + 'onnxruntime_cxx_inline.h': 'c41b9a11aac46851f68f4bd888bb2444a0e1eb6b37eadb41bf15f5154b48ea31',
    ORT + 'onnxruntime_float16.h': '6c4db3d1954b266c4febe413351af86cac922e3448538e7e452e51d646e56a4a',
    ORT + 'onnxruntime_session_options_config_keys.h': '301c304aa5082549148b59d64e70df951821eb111aea0690e3edeef6994ef182',
    ORT + 'onnxruntime_run_options_config_keys.h': '973037c14b0f1af0854ae25cd676776f79dbcfd2a6791f6ee17a5374c2a62550',
    'https://github.com/opencv/opencv/archive/refs/tags/4.6.0.tar.gz':
        '1ec1cba65f9f20fe5a41fda1586e01c70ea0c9a6d7b67c9e13edf0cfe2239277',
}

FALLBACKS = {
    'https://deb.debian.org/debian/pool/main/e/eigen3/libeigen3-dev_3.4.0-4_all.deb':
        ['https://snapshot.debian.org/archive/debian/20250101T000000Z/pool/main/e/eigen3/libeigen3-dev_3.4.0-4_all.deb'],
}


def _extract(archive, target):
    try:
        archive.extractall(target, filter='data')
    except TypeError:
        archive.extractall(target)


def unpack_deb(data, target):
    if data[:8] != b'!<arch>\n':
        raise ValueError('Invalid deb archive')
    offset = 8
    while offset < len(data):
        header = data[offset:offset + 60]
        size = int(header[48:58])
        name = header[:16].decode().strip().rstrip('/')
        payload = data[offset + 60:offset + 60 + size]
        offset += 60 + size + size % 2
        if name.startswith('data.tar'):
            if name.endswith('.xz'):
                payload = lzma.decompress(payload)
            elif name.endswith('.gz'):
                payload = gzip.decompress(payload)
            with tarfile.open(fileobj=io.BytesIO(payload)) as archive:
                _extract(archive, target)
            return
    raise ValueError('No data archive')


def main():
    downloads = ROOT / 'downloads'
    downloads.mkdir(parents=True, exist_ok=True)
    records = []
    for url, digest in PINS.items():
        path = downloads / url.rsplit('/', 1)[1]
        if url.endswith('/4.6.0.tar.gz'):
            path = downloads / 'opencv-4.6.0.tar.gz'
        bundled = BUNDLED / path.name
        if bundled.is_file() and hashlib.sha256(bundled.read_bytes()).hexdigest() == digest:
            path.write_bytes(bundled.read_bytes())
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            for source in [url] + FALLBACKS.get(url, []):
                try:
                    with urllib.request.urlopen(source, timeout=300) as response:
                        path.write_bytes(response.read())
                    break
                except OSError as error:
                    print(f'{source}: {error}', file=sys.stderr)
        data = path.read_bytes()
        got = hashlib.sha256(data).hexdigest()
        if got != digest:
            raise SystemExit(f'{url}: SHA-256 {got} does not match the pin {digest}')
        records.append({'url': url, 'file': path.name, 'size': len(data), 'sha256': got})
        if path.suffix == '.deb':
            unpack_deb(data, ROOT)
        elif path.suffix == '.h':
            headers = ROOT / 'onnxruntime' / 'include'
            headers.mkdir(parents=True, exist_ok=True)
            (headers / path.name).write_bytes(data)
        else:
            with tarfile.open(path) as archive:
                _extract(archive, ROOT)
    (ROOT / 'download-manifest.json').write_text(json.dumps(records, indent=2) + '\n')
    print(json.dumps({'files': len(records), 'bytes': sum(r['size'] for r in records), 'root': str(ROOT)}))


if __name__ == '__main__':
    main()
