"""Local, checksum-verified official Arduino ESP32 3.3.12 subset for ESP32 only.
Advanced Windows alternative to build.ps1 -InstallCore. Uses the same official
archives but downloads only the classic ESP32 target tools. Does not provide
RISC-V/S2/S3 toolchains, does not modify the official package index, and never
uploads firmware or opens a serial port. Python standard library only.

Usage: python scripts/setup_esp32_subset.py [--arduino-cli PATH] [--build-root PATH]
Then:  ./scripts/build.ps1 [-BuildRoot PATH]
For a deeply nested project on Windows, choose a short build directory. It must
be the same directory in both commands. No existing project files are deleted.
"""
import concurrent.futures
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request
import zipfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--arduino-cli', default=shutil.which('arduino-cli') or
                    'C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe')
parser.add_argument('--build-root', type=Path, default=Path(__file__).resolve().parent.parent / '.build')
args = parser.parse_args()
BUILD = args.build_root.resolve()
DATA = BUILD / 'arduino-data'
DOWNLOADS = BUILD / 'downloads' / 'packages'
BUILD.mkdir(parents=True, exist_ok=True)
config = BUILD / 'arduino-cli.yaml'
quote = lambda path: "'" + str(path).replace("'", "''") + "'"
config.write_text('board_manager:\n  additional_urls:\n'
                  '    - https://espressif.github.io/arduino-esp32/package_esp32_index.json\n'
                  'directories:\n  data: ' + quote(DATA) + '\n  downloads: ' + quote(BUILD / 'downloads') +
                  '\n  user: ' + quote(BUILD / 'sketchbook') + '\n', encoding='utf-8')
subprocess.run([args.arduino_cli, '--config-file', str(config), 'core', 'update-index'], check=True)
PACKAGE = json.loads((DATA / 'package_esp32_index.json').read_text(encoding='utf-8'))['packages'][0]
CORE = next(p for p in PACKAGE['platforms'] if p['version'] == '3.3.12')
SELECTED = {'esp-x32', 'xtensa-esp-elf-gdb', 'esptool_py', 'mkspiffs', 'mklittlefs', 'esp32-libs'}
ENTRIES = [(CORE, DATA / 'packages/esp32/hardware/esp32/3.3.12')]
for dep in CORE['toolsDependencies']:
    if dep['name'] not in SELECTED:
        continue
    tool = next(t for t in PACKAGE['tools'] if t['name'] == dep['name'] and t['version'] == dep['version'])
    system = next((s for s in tool['systems'] if s['host'] == 'x86_64-mingw32'), None)
    if system is None:
        system = next(s for s in tool['systems'] if s['host'] == 'i686-mingw32')
    ENTRIES.append((system, DATA / 'packages/esp32/tools' / dep['name'] / dep['version']))

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def prepare(entry):
    spec, dest = entry
    archive = DOWNLOADS / spec['archiveFileName']
    expected = spec['checksum'].split(':', 1)[1].lower()
    if not archive.exists() or digest(archive) != expected:
        temp = archive.with_suffix(archive.suffix + '.subset-part')
        print('Downloading ' + archive.name, flush=True)
        with urllib.request.urlopen(spec['url'], timeout=120) as src, temp.open('wb') as dst:
            shutil.copyfileobj(src, dst, 1024 * 1024)
        if digest(temp) != expected:
            raise RuntimeError('Checksum mismatch: ' + archive.name)
        temp.replace(archive)
    print('Verified ' + archive.name, flush=True)
    with zipfile.ZipFile(archive) as z:
        files = [i for i in z.infolist() if not i.is_dir()]
        firsts = {i.filename.split('/')[0] for i in files}
        strip = next(iter(firsts)) + '/' if len(firsts) == 1 and all('/' in i.filename for i in files) else ''
        for item in files:
            name = item.filename[len(strip):]
            target = (dest / name).resolve()
            target.relative_to(dest.resolve())
            # Python extended-length Windows paths support long official SDK
            # headers without changing the user's LongPathsEnabled registry.
            target = Path('\\\\?\\' + str(target))
            target.parent.mkdir(parents=True, exist_ok=True)
            with z.open(item) as src, target.open('wb') as dst:
                shutil.copyfileobj(src, dst)
    print('Installed ' + str(dest.relative_to(BUILD)), flush=True)
    return {'url':spec['url'], 'sha256':expected, 'destination':str(dest.relative_to(BUILD))}

if __name__ == '__main__':
    DOWNLOADS.mkdir(parents=True, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(prepare, ENTRIES))
    (BUILD / 'esp32-subset-manifest.json').write_text(json.dumps({'note':'Official archive subset; ESP32 target only. No RISC-V/S2/S3 packages.', 'packages':results}, indent=2))

