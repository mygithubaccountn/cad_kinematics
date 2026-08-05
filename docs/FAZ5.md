# Faz 5 — Prismatic / SCARA

## Durum

**Tamamlandı.** `CURRENT_PHASE = 5`

## Ne değişti

`analyze` artık `include_prismatic=True`. Benzer yarıçaplı parallel guide silindirleri + contact → prismatic hipotezi. Revolute shaft-hole yoksa prismatic öne geçer.

## Fixture

`fixtures/scara.synthetic.json` — R-R-P

## Tasarım

Prismatic’i revolute ile aynı multi-hypothesis çerçevede tutmak; ayrı “SCARA özel kod” yok.

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz5.py -q
```
