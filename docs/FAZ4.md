# Faz 4 — Validation

## Durum

**Tamamlandı.** `CURRENT_PHASE = 4`

## Kontroller

- Ağaç: tek base, orphan yok
- Pivot bbox yakınlığı
- Rest-pose origin tutarlılığı
- Small-angle revolute smoke (±α, eksen mesafesi sabit)

## Fixtures

- `fixtures/serial_3dof.synthetic.json`
- `fixtures/scara.synthetic.json` (prismatic Faz 5’te aktif)

## Tasarım

Validation ayrı modül; joint skorunu “görsel OK” ile karıştırmaz. Başarı = insan müdahalesiz doğru kinematik, demo sayısı değil.

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz4.py -q
```
