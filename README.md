# CAD → Godot Robot Pipeline

**Current phase: 6** — docs in `docs/FAZ0.md` … `docs/FAZ6.md`

## Önemli: iki ayrı Python

| Ne yapıyorsun | Komut |
|---------------|--------|
| Sentetik / test | `./run.sh …` (sistem Python; FreeCAD PYTHONPATH **yok**) |
| Gerçek STEP | `./run_with_freecad.sh …` (FreeCAD’in kendi Python 3.11) |

Shell’de daha önce şunu yaptıysan:

```bash
export PYTHONPATH="/Applications/FreeCAD.app/..."
```

sistem `python` / `python3` **bozulur** (numpy hatası). Düzelt:

```bash
unset PYTHONPATH
```

## Çalışan komutlar (yorum satırı yapıştırma)

```bash
cd "/Users/nisa/Desktop/new cad project"
unset PYTHONPATH

./run.sh --phase-info
./run.sh run fixtures/serial_3dof.synthetic.json --out out/run
./run.sh run fixtures/scara.synthetic.json --out out/scara

./run_with_freecad.sh run "/Users/nisa/Desktop/robot_assembly.stp" --out out/step

PYTHONPATH=src:. python3 -m pytest tests/test_faz0.py tests/test_faz1.py tests/test_faz2.py tests/test_faz3.py tests/test_faz4.py tests/test_faz5.py tests/test_faz6.py -q
```

`/path/to/robot.step` bir örnek yer tutucudur — gerçek dosya yolu kullan.

## Layout

```
src/{importer,geometry,joints,hierarchy,exporter,validation,common}
schemas/  fixtures/  godot/  docs/
pipeline.py  run.sh  run_with_freecad.sh
```
