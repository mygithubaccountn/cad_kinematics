# CAD → Kinematics → Godot — Architecture (v2)

> Hedef: üreticiden bağımsız, Godot’da manuel düzeltme gerektirmeyen,
> çok-kanıtlı kinematik çıkarım. Mesh export ikincildir.

## Öncelikler

1. **Doğruluk** — yanlış pivot/eksen kabul edilmez; Godot’da düzeltilmez  
2. **Güvenilirlik** — DecisionTrace + validation gate  
3. **Genellik** — robota / markaya özel kural yok  
4. **Performans** — cache ve tek FreeCAD oturumu; doğruluktan ödün yok  

## Kabul edilmeyenler

- Godot’da pivot / parent / eksen elle düzeltme  
- Tek heuristiğe güvenmek  
- Aynı STEP’i pipeline içinde tekrar tekrar `Import.insert`  
- Değişmeyen aşamaları her `run`’da yeniden hesaplamak  
- Geçici “şu robot için if” yamaları  

## Veri akışı

```
STEP
  → S0 AssemblyIR          (parts, world-baked bbox/placement)
  → S1 FeatureGraph        (cylinders, contacts, adjacency, clusters)
  → S2 JointHypothesis[]   (multi-evidence scores + traces)
  → S3 ResolvedJoint[]     (selection + pivot/axis refinement)
  → S4 KinematicTree       (base, welds/orphans, link_world, joints)
  → S5 RobotDesc + GLB     (same frame for mesh & joints)
  → S6 ValidationReport    (gate)
```

Godot / viewer yalnızca `robot.json` + `meshes/*.glb` tüketir.

### Çerçeveler

- İç hesap: **metre, CAD Z-up**, translation-only link frames (rest pose = STEP).  
- Upright / Y-up dönüşümü **yalnızca S5’te**, mesh ve joint **birlikte**.  

## Aşama sözleşmeleri

| Stage | Girdi | Çıktı | Yeniden çalıştırma |
|-------|--------|--------|---------------------|
| S0 Ingest | STEP path | `assembly_ir.json` | source hash değişince |
| S1 Features | IR | `features.json` | IR hash değişince |
| S2 Hypotheses | IR + features | `joint_hypotheses.json` | features/tol değişince |
| S3 Select+Axis | hypotheses | `joints_selected.json`, `resolved_axes.json` | hyp/tol değişince |
| S4 Hierarchy | resolved + features | `kinematic_tree.json` | joints değişince |
| S5 Package | tree + IR | `robot.json`, `meshes/` | tree değişince; `--remesh` zorla |
| S6 Validate | tree + robot | `validation_report.json` | her paket sonrası |

`manifest.json`: her artifact için `inputs_hash`, `algorithm_version`, `created_at`.

## DecisionTrace (zorunlu)

Her joint kararı (seçilen ve reddedilen önemli adaylar):

- subject (part çifti / cluster)  
- evidence[]: `{name, score, detail}`  
- rejected[]: neden  
- **chosen** `{origin, axis, method, confidence}` (world, CAD Z-up)  
- **runner_up** `{name, origin, axis, score}` (2. en iyi aday)  
- notes[]  

Pivot/axis: `axis_consensus-1` — adaylar (`cluster_median`, `cyl_overlap_mid`, `shaft_hole_mid`, `contact_on_axis`, `hypothesis`); agreement skoru; yakınsa blend, değilse top + reject diğerleri.

Artifact’lar: `decision_traces.json` (`selected` + `rejected_notable`), `decision_trace.json` (resolved axis traces), `resolved_axes.json`.

Yanlış tahmin → trace ile algoritma geliştirilir.

## Joint / pivot politikası

Çok-kanıt (örnek, merkezi `Tolerances`):

- concentric clusters, shaft-in-hole, contact ring  
- **AABB proximity filtresi** (uzak eşeksenli sahte joint’leri kes)  
- adjacency, mate hints, volume direction  

Seçim: endüstriyel kollar için **base’den seri zincir** önceliği; kalan yüksek conf kenarlar cycle’sız eklenir.  
Orphan: kör base weld yok — proximity + contact + volume + host-in-tree skoru; güçlüyse merge, zayıfsa `suspicious_orphans`.

## Performans politikası

- FreeCAD: **tek shared document** / process (import + features + tessellate)  
- Silindir: min r/h + per-part cap (vida gürültüsü)  
- Mesh: preview deflection varsayılan; yüksek kalite ayrı flag  
- CLI: `run` (dirty stages) | `run --force` | `stage <name>` | `status`

```bash
# Full run (skips fresh stages on re-run)
./run_with_freecad.sh run robot.step --out out/step
./run.sh run fixtures/serial_3dof.synthetic.json --out out/syn

# Recompute everything
./run_with_freecad.sh run robot.step --out out/step --force

# Invalidate joints+downstream (keep ingest/features cache), then run
./run_with_freecad.sh run robot.step --out out/step --from-stage joints

# Single stage (deps must already exist)
./run_with_freecad.sh stage joints --out out/step --force

# Cache table
./run_with_freecad.sh status --out out/step
```

`manifest.json` stores per-stage `inputs_hash`, `algorithm_version`, `status`.  
Bump `ALGORITHM_VERSIONS` in `src/common/manifest.py` when stage logic changes.

Hedef: ~10 parçalı montaj **birkaç dakika**; 100+ parça “heavy” profil.

## Modül haritası (hedef layout)

```
src/
  ingest/          # S0
  features/        # S1  (FreeCAD yüzleri burada)
  joints/          # S2–S3
  hierarchy/       # S4
  package/         # S5 scene + mesh
  validation/      # S6
  common/          # models, tolerances, frames, math, trace, manifest
  freecad/         # session + backend only
```

Eski `pipeline/` ağacı ve çift yollar kaldırılır / tek `src` altında birleştirilir.

## Uygulama sırası

1. **Manifest + incremental CLI** (performans / DX, doğruluğu bozmaz)  
2. **Pivot/axis consensus + tam DecisionTrace** (ana doğruluk sorunu)  
3. **Validation gate (confidence-based)** — kritik fail, soft warning  
4. **Fixture matrisi** (serial / SCARA / 6DOF, üreticisiz)  
5. Mesh kalitesi / heavy profil  

## Validation gate (confidence-based)

- **Hard fail (`ok=false`)**: kritik geometri — kopuk child, pivot/mesh tutarsızlığı, FK/transform kırığı, degenerate axis, eksik mesh  
- **Warning**: düşük conf joint, orphan, az movable joint, eksik kanıt, soft FK/smoke drift  
- Report: `overall_confidence`, `unresolved_parts`, `suspicious_joints`, `warnings[]`  

Düşük kaliteli STEP → pipeline yine `robot.json` üretir; güven skoru ile “şüpheli ama kullanılabilir” sinyali verir.

## Godot sözleşmesi (özet)

```json
{
  "frame": "cad_z_up",
  "base_link": "...",
  "links": [{ "id", "mesh", "part_ids" }],
  "joints": [{ "parent", "child", "type", "origin", "axis", "confidence" }]
}
```

Rest pose açıları 0. Hareket `origin` + `axis` etrafında. Kritik validation fail → ship etme; warning’li sonuçlar confidence ile tüketilir.
