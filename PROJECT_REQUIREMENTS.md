{\rtf1\ansi\ansicpg1254\cocoartf2820
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\froman\fcharset0 Times-Bold;\f1\froman\fcharset0 Times-Roman;\f2\fmodern\fcharset0 Courier;
\f3\froman\fcharset0 Times-Italic;}
{\colortbl;\red255\green255\blue255;\red0\green0\blue0;\red109\green109\blue109;}
{\*\expandedcolortbl;;\cssrgb\c0\c0\c0;\cssrgb\c50196\c50196\c50196;}
{\*\listtable{\list\listtemplateid1\listhybrid{\listlevel\levelnfc23\levelnfcn23\leveljc0\leveljcn0\levelfollow0\levelstartat1\levelspace360\levelindent0{\*\levelmarker \{disc\}}{\leveltext\leveltemplateid1\'01\uc0\u8226 ;}{\levelnumbers;}\fi-360\li720\lin720 }{\listname ;}\listid1}
{\list\listtemplateid2\listhybrid{\listlevel\levelnfc23\levelnfcn23\leveljc0\leveljcn0\levelfollow0\levelstartat1\levelspace360\levelindent0{\*\levelmarker \{disc\}}{\leveltext\leveltemplateid101\'01\uc0\u8226 ;}{\levelnumbers;}\fi-360\li720\lin720 }{\listname ;}\listid2}
{\list\listtemplateid3\listhybrid{\listlevel\levelnfc23\levelnfcn23\leveljc0\leveljcn0\levelfollow0\levelstartat1\levelspace360\levelindent0{\*\levelmarker \{disc\}}{\leveltext\leveltemplateid201\'01\uc0\u8226 ;}{\levelnumbers;}\fi-360\li720\lin720 }{\listname ;}\listid3}
{\list\listtemplateid4\listhybrid{\listlevel\levelnfc23\levelnfcn23\leveljc0\leveljcn0\levelfollow0\levelstartat1\levelspace360\levelindent0{\*\levelmarker \{disc\}}{\leveltext\leveltemplateid301\'01\uc0\u8226 ;}{\levelnumbers;}\fi-360\li720\lin720 }{\listname ;}\listid4}
{\list\listtemplateid5\listhybrid{\listlevel\levelnfc23\levelnfcn23\leveljc0\leveljcn0\levelfollow0\levelstartat1\levelspace360\levelindent0{\*\levelmarker \{disc\}}{\leveltext\leveltemplateid401\'01\uc0\u8226 ;}{\levelnumbers;}\fi-360\li720\lin720 }{\listname ;}\listid5}}
{\*\listoverridetable{\listoverride\listid1\listoverridecount0\ls1}{\listoverride\listid2\listoverridecount0\ls2}{\listoverride\listid3\listoverridecount0\ls3}{\listoverride\listid4\listoverridecount0\ls4}{\listoverride\listid5\listoverridecount0\ls5}}
\paperw11900\paperh16840\margl1440\margr1440\vieww28900\viewh16100\viewkind0
\deftab720
\pard\pardeftab720\sa321\partightenfactor0

\f0\b\fs48 \cf0 \expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Mimari Tasar\uc0\u305 m: CAD \u8594  Kinematik \'c7\u305 kar\u305 m \u8594  Godot Pipeline'\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Kodlamaya ge\'e7meden \'f6nce mimariyi netle\uc0\u351 tirelim. A\u351 a\u287 \u305 daki tasar\u305 m, senin belirtti\u287 in \'f6ncelik s\u305 ras\u305 na (kinematik do\u287 ruluk > genellenebilirlik > a\'e7\u305 klanabilirlik > performans > mesh) sad\u305 k kalacak \u351 ekilde kurguland\u305 .\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 0. Temel Felsefe\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 En kritik ger\'e7ek \uc0\u351 u: 
\f0\b d\'fcz bir STEP dosyas\uc0\u305  (AP203/AP214) joint bilgisi i\'e7ermez.
\f1\b0  Sadece geometri + montaj hiyerar\uc0\u351 isi (par\'e7a yerle\u351 imleri) ta\u351 \u305 r. Yani "revolute mi prismatic mi" sorusunun cevab\u305  dosyada yaz\u305 l\u305  de\u287 il \'97 
\f0\b geometriden \'e7\uc0\u305 kar\u305 lmas\u305  gereken bir \'e7\u305 kar\u305 md\u305 r.
\f1\b0 \
Bu y\'fczden sistemin kalbi bir "joint reader" de\uc0\u287 il, bir 
\f0\b "geometrik mating detector + DOF (serbestlik derecesi) s\uc0\u305 n\u305 fland\u305 r\u305 c\u305 s\u305 "
\f1\b0  olmal\uc0\u305 . Mimari buna g\'f6re kurulmal\u305 : her karar, \'f6l\'e7\'fclebilir bir geometrik kan\u305 ta dayanmal\u305  (contact y\'fczeyi, eksen \'e7ak\u305 \u351 mas\u305 , yar\u305 \'e7ap e\u351 le\u351 mesi), asla isimlendirmeye veya heuristic tahmine de\u287 il.\
\uc0\u304 kinci \'f6nemli nokta: E\u287 er kullan\u305 c\u305  FreeCAD native Assembly (Assembly4 / yeni Assembly workbench) ile haz\u305 rlanm\u305 \u351  bir 
\f2\fs26 .FCStd
\f1\fs24  verirse, orada 
\f0\b ger\'e7ek joint tan\uc0\u305 mlar\u305  (LCS'ler, eksen k\u305 s\u305 tlar\u305 ) zaten mevcuttur.
\f1\b0  Bu ground truth'tur ve varsa \'f6ncelikli kullan\uc0\u305 lmal\u305 . Sadece ham STEP geldi\u287 inde geometrik \'e7\u305 kar\u305 ma d\'fc\u351 \'fclmeli. Mimari bu iki kayna\u287 \u305  ayn\u305  ara formata (IR) normalize etmeli.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 1. \'dcst D\'fczey Pipeline (A\uc0\u351 amalar)\
\pard\pardeftab720\partightenfactor0

\f2\b0\fs26 \cf0 [0] Import & Normalize\
[1] Part Segmentation\
[2] Surface Feature Extraction   (par\'e7a ba\uc0\u351 \u305 na y\'fczey imzalar\u305 )\
[3] Native Joint Extraction      (varsa \'97 FCStd/Assembly4 ground truth)\
[4] Contact/Mating Graph         (yoksa geometrik \'e7\uc0\u305 kar\u305 m)\
[5] Joint Classification (DOF)   (revolute / prismatic / fixed / di\uc0\u287 er)\
[6] Kinematic Tree Construction  (root se\'e7imi, spanning tree, d\'f6ng\'fc \'e7\'f6z\'fcm\'fc)\
[7] Pivot & Axis Resolution      (kesin geometrik parametreler)\
[8] Validation & Confidence      (her karar i\'e7in gerek\'e7e + skor)\
[9] Godot Export (IR \uc0\u8594  JSON)\
[10] Mesh Export (glTF, en son a\uc0\u351 ama)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Her a\uc0\u351 ama 
\f0\b saf bir fonksiyon
\f1\b0  gibi tasarlanmal\uc0\u305 : girdi = \'f6nceki a\u351 aman\u305 n \'e7\u305 kt\u305 s\u305  + o a\u351 amaya \'f6zel parametreler, \'e7\u305 kt\u305  = de\u287 i\u351 mez (immutable) bir ara veri yap\u305 s\u305 . Bu, hem test edilebilirli\u287 i hem de incremental caching'i do\u287 rudan m\'fcmk\'fcn k\u305 lar (a\u351 a\u287 \u305 da detay var).\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 2. Ara Format (IR \'97 Intermediate Representation)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 B\'fct\'fcn a\uc0\u351 amalar aras\u305  veri, tek bir merkezi \u351 ema \'fczerinden akmal\u305 . \'d6neri (kavramsal, dile ba\u287 \u305 ms\u305 z):\
\pard\pardeftab720\partightenfactor0

\f2\fs26 \cf0 AssemblyIR\
\uc0\u9500 \u9472 \u9472  parts: [PartNode]\
\uc0\u9474      \u9500 \u9472 \u9472  id, name, shape_hash\
\uc0\u9474      \u9500 \u9472 \u9472  local_transform (CAD d\'fcnya koordinat\u305 nda)\
\uc0\u9474      \u9500 \u9472 \u9472  bounding_box, volume, center_of_mass\
\uc0\u9474      \u9492 \u9472 \u9472  faces: [FaceFeature]\
\uc0\u9474            \u9500 \u9472 \u9472  surface_type (plane/cylinder/cone/sphere/torus)\
\uc0\u9474            \u9500 \u9472 \u9472  params (axis, radius, origin, normal...)\
\uc0\u9474            \u9492 \u9472 \u9472  area, adjacency (edge/loop bilgisi)\
\uc0\u9474 \
\uc0\u9500 \u9472 \u9472  contacts: [ContactEdge]\
\uc0\u9474      \u9500 \u9472 \u9472  part_a, part_b\
\uc0\u9474      \u9500 \u9472 \u9472  matching_faces: [(face_a, face_b, match_type, tolerance)]\
\uc0\u9474      \u9492 \u9472 \u9472  evidence_score\
\uc0\u9474 \
\uc0\u9500 \u9472 \u9472  joints: [JointCandidate]\
\uc0\u9474      \u9500 \u9472 \u9472  part_parent, part_child\
\uc0\u9474      \u9500 \u9472 \u9472  type: revolute | prismatic | fixed | cylindrical | unknown\
\uc0\u9474      \u9500 \u9472 \u9472  axis (point + direction, d\'fcnya koordinat\u305 nda, KES\u304 N geometrik)\
\uc0\u9474      \u9500 \u9472 \u9472  pivot (nokta)\
\uc0\u9474      \u9500 \u9472 \u9472  limits (varsa, contact geometrisinden tahmini)\
\uc0\u9474      \u9500 \u9472 \u9472  confidence, evidence[] (gerek\'e7e listesi)\
\uc0\u9474      \u9492 \u9472 \u9472  source: "native" | "inferred"\
\uc0\u9474 \
\uc0\u9492 \u9472 \u9472  tree: KinematicTree (root + parent-child graph, d\'f6ng\'fc raporlar\u305 )\
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Bu IR, 
\f2\fs26 [9]
\f1\fs24  ve 
\f2\fs26 [10]
\f1\fs24  a\uc0\u351 amalar\u305 n\u305 n 
\f0\b tek girdisi
\f1\b0 . Godot exportu bu \uc0\u351 emadan t\'fcretilir, mesh exportu da ba\u287 \u305 ms\u305 z olarak bu \u351 emadan t\'fcretilir \'97 yani mesh \'fcretimi kinematik \'e7\u305 kar\u305 m\u305 na hi\'e7bir \u351 ekilde kar\u305 \u351 maz (senin istedi\u287 in ayr\u305 m tam olarak bu).\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 3. En Kritik A\uc0\u351 ama: Contact Detection & Joint Classification\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Bunu detayland\uc0\u305 r\u305 yorum \'e7\'fcnk\'fc projenin ba\u351 ar\u305 s\u305  bununla \'f6l\'e7\'fclecek.\
\pard\pardeftab720\sa240\partightenfactor0

\f0\b \cf0 3a. Y\'fczey imzas\uc0\u305  \'e7\u305 kar\u305 m\u305  (A\u351 ama 2)
\f1\b0  Her par\'e7an\uc0\u305 n B-Rep y\'fczeyleri (OCC/FreeCAD 
\f2\fs26 Part.Face
\f1\fs24 ) tek tek analiz edilir; her y\'fczey i\'e7in tip + parametreler kanonik forma indirgenir (silindir \uc0\u8594  eksen do\u287 rusu + yar\u305 \'e7ap; d\'fczlem \u8594  normal + nokta; vs.). Bu, montajdaki 
\f3\i her
\f1\i0  par\'e7a \'e7ifti i\'e7in O(n\'b2) kaba kuvvet kar\uc0\u351 \u305 la\u351 t\u305 rma de\u287 il \'97 mekansal indeksleme (bounding box overlap + yar\u305 \'e7ap/eksen hash'leme) ile filtrelenmi\u351  bir aday listesi \'fcretir.\

\f0\b 3b. Mating graph (A\uc0\u351 ama 4)
\f1\b0  \uc0\u304 ki par\'e7a aras\u305 nda 
\f0\b e\uc0\u351  eksenli silindirik y\'fczey \'e7ifti
\f1\b0  (shaft-in-hole) bulunursa, bu g\'fc\'e7l\'fc bir "buradan hareket ge\'e7iyor" sinyalidir \'97 ama tek ba\uc0\u351 \u305 na joint tipini belirlemez (cylindrical joint: hem d\'f6nebilir hem kayabilir). Tipi belirleyen \u351 ey, 
\f0\b o eksene ek olarak var olan di\uc0\u287 er k\u305 s\u305 tlard\u305 r:
\f1\b0 \
\pard\tx220\tx720\pardeftab720\li720\fi-720\partightenfactor0
\ls1\ilvl0\cf0 \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Silindirik e\uc0\u351 -eksen + eksene dik iki d\'fczlemsel temas (omuz/flan\u351 ) \u8594  
\f0\b translasyon engellenir \uc0\u8594  revolute
\f1\b0 \
\ls1\ilvl0\kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Sadece silindirik e\uc0\u351 -eksen, eksenel temas yok \u8594  
\f0\b cylindrical/prismatic
\f1\b0  (context'e g\'f6re)\
\ls1\ilvl0\kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 \uc0\u304 ki paralel d\'fczlem + k\u305 lavuz (rail) y\'fczeyi, d\'f6nme k\u305 s\u305 tl\u305  \u8594  
\f0\b prismatic
\f1\b0 \
\ls1\ilvl0\kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 T\'fcm y\'fczeyler tam \'f6rt\'fc\uc0\u351 \'fcyor, relatif hareket serbestlik derecesi 0 \u8594  
\f0\b fixed
\f1\b0 \
\pard\pardeftab720\sa240\partightenfactor0
\cf0 Bu, klasik "contact graph \uc0\u8594  DOF elimination" yakla\u351 \u305 m\u305  (assembly constraint solving literat\'fcr\'fcndeki reverse-engineering y\'f6ntemlerine benzer). Kritik nokta: 
\f0\b eksen ve pivot, contact y\'fczeyinin kendi geometrik parametresinden okunur
\f1\b0  (silindirin ekseni ve merkezi) \'97 fit/optimizasyon ile "tahmin edilmez". Bu, senin bahsetti\uc0\u287 in "yanl\u305 \u351  eksen tahmini" sorununu k\'f6kten azalt\u305 r \'e7\'fcnk\'fc tahmin de\u287 il \'f6l\'e7\'fcmd\'fcr.\
\pard\pardeftab720\sa240\partightenfactor0

\f0\b \cf0 3c. Belirsizlik durumlar\uc0\u305 
\f1\b0  Baz\uc0\u305  montajlarda contact y\'fczeyleri temiz olmayabilir (gap/interference toleranslar\u305 , chamfer'lar). Bu y\'fczden her e\u351 le\u351 me bir 
\f0\b tolerans penceresi + confidence score
\f1\b0  ile \'fcretilmeli, ve birden fazla aday varsa hepsi IR'de saklan\uc0\u305 p A\u351 ama 8'de raporlanmal\u305  \'97 "sessizce yanl\u305 \u351  se\'e7im yapma" yerine "bu joint'i %62 g\'fcvenle revolute olarak s\u305 n\u305 fland\u305 rd\u305 m, \'e7\'fcnk\'fc X ve Y" diyebilmeli.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 4. Kinematik A\uc0\u287 a\'e7 Kurulumu (A\u351 ama 6)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Contact graph bir 
\f0\b graf
\f1\b0 t\uc0\u305 r, a\u287 a\'e7 de\u287 il (kapal\u305  zincirler / paralel mekanizmalar olabilir \'97 \'f6rn. paralel robot kollar\u305 ). Root se\'e7imi: genelde en b\'fcy\'fck hacimli / d\'fcnyaya sabit (fixed joint'i olmayan veya en \'e7ok ba\u287 lant\u305 s\u305  olan) par\'e7a baz al\u305 n\u305 r, sonra BFS/spanning tree ile parent-child \'e7\u305 kar\u305 l\u305 r. D\'f6ng\'fc tespit edilirse (spanning tree d\u305 \u351 \u305  kenar kal\u305 rsa), bu 
\f0\b a\'e7\uc0\u305 k\'e7a raporlan\u305 r
\f1\b0  \'97 "bu bir seri zincir de\uc0\u287 il, kapal\u305  d\'f6ng\'fc i\'e7eriyor, ek constraint mekanizmas\u305  var" gibi. Bu \u351 effafl\u305 k, SCARA'dan 6 eksenliye, paralel mekanizmalara kadar genellenebilirli\u287 i garanti eden k\u305 s\u305 m.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 5. A\'e7\uc0\u305 klanabilirlik (A\u351 ama 8)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Her 
\f2\fs26 JointCandidate
\f1\fs24  bir 
\f2\fs26 evidence[]
\f1\fs24  listesi ta\uc0\u351 \u305 r: hangi y\'fczey \'e7iftleri, hangi tolerans, hangi DOF elemesi kullan\u305 ld\u305 . Reddedilen adaylar da (neden reddedildi\u287 iyle birlikte) saklan\u305 r. Bu bir log de\u287 il, 
\f0\b IR'nin par\'e7as\uc0\u305 
\f1\b0  \'97 yani export format\uc0\u305 na da, debug aray\'fcz\'fcne de ayn\u305  kaynaktan beslenir.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 6. Incremental / Cache Mimarisi\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Build-system mant\uc0\u305 \u287 \u305  (Bazel/Make tarz\u305  content-addressed caching):\
\pard\tx220\tx720\pardeftab720\li720\fi-720\partightenfactor0
\ls2\ilvl0\cf0 \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Her a\uc0\u351 ama fonksiyonu 
\f2\fs26 hash(girdi_IR + parametreler + algoritma_versiyonu)
\f1\fs24  anahtar\uc0\u305 yla diske cache'lenir.\
\ls2\ilvl0\kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 A\uc0\u351 ama 5'teki (joint classification) algoritmay\u305  de\u287 i\u351 tirdi\u287 inde, sadece A\u351 ama 5\u8594 10 yeniden \'e7al\u305 \u351 \u305 r; A\u351 ama 0-4 (geometri parse, y\'fczey \'e7\u305 kar\u305 m\u305  \'97 pahal\u305  OCC i\u351 lemleri) cache'ten okunur.\
\ls2\ilvl0\kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Bu, geli\uc0\u351 tirme d\'f6ng\'fcs\'fcn\'fc saniyeler mertebesine indirir \'e7\'fcnk\'fc en pahal\u305  ad\u305 m (STEP parse + B-Rep analizi) tekrar tekrar \'e7al\u305 \u351 maz.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 7. Godot Export (A\uc0\u351 ama 9)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 IR'den t\'fcretilen basit, kay\uc0\u305 ps\u305 z bir JSON: her 
\f2\fs26 PartNode
\f1\fs24  i\'e7in Godot-uzay\uc0\u305 na d\'f6n\'fc\u351 t\'fcr\'fclm\'fc\u351  transform (CAD sa\u287 -el / Z-up \u8594  Godot sol-el / Y-up d\'f6n\'fc\u351 \'fcm\'fc tek bir merkezi fonksiyonda yap\u305 l\u305 r, da\u287 \u305 n\u305 k de\u287 il), her 
\f2\fs26 Joint
\f1\fs24  i\'e7in pivot + eksen + tip + limit. Godot taraf\uc0\u305  sadece bunu okuyup 
\f2\fs26 Skeleton3D
\f1\fs24 /
\f2\fs26 Joint3D
\f1\fs24  (veya senin tercih edece\uc0\u287 in node yap\u305 s\u305 ) kurar \'97 hi\'e7bir transform d\'fczeltmesi yapmaz. Bu prensip mimarinin en kat\u305  kural\u305 : 
\f0\b Godot taraf\uc0\u305  "dumb reader" olmal\u305 .
\f1\b0 \
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 8. Teknoloji \'d6nerisi\
\pard\tx220\tx720\pardeftab720\li720\fi-720\partightenfactor0
\ls3\ilvl0
\fs24 \cf0 \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Geometri \'e7ekirde\uc0\u287 i:
\f1\b0  FreeCAD Python API (
\f2\fs26 Part
\f1\fs24 , 
\f2\fs26 TopoShape
\f1\fs24 ), \'e7\'fcnk\'fc hem native Assembly4/Assembly joint verisine eri\uc0\u351 ebiliyor hem de alt\u305 nda OCC var. Gerekirse d\'fc\u351 \'fck seviye B-Rep sorgular\u305  i\'e7in 
\f2\fs26 pythonocc-core
\f1\fs24 'a inilebilir \'97 hibrit, senin de belirtti\uc0\u287 in gibi.\
\ls3\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Test korpusu:
\f1\b0  En az 3-4 farkl\uc0\u305  yap\u305  (SCARA, 6 eksenli seri, gripper/paralel mekanizma, basit mente\u351 e) \'97 regression test seti olarak, her algoritma de\u287 i\u351 ikli\u287 inde bunlara kar\u351 \u305  otomatik do\u287 rulama.\
\pard\pardeftab720\partightenfactor0
\cf3 \strokec3 \
\pard\pardeftab720\sa240\partightenfactor0
\cf0 \strokec2 Bu iskelet \'fczerinde ilerlemek istersen, \'f6nerim: \'f6nce 
\f0\b A\uc0\u351 ama 0-2 + IR \u351 emas\u305 n\u305 
\f1\b0  somut Python veri s\uc0\u305 n\u305 flar\u305  olarak kodlay\u305 p, elimizdeki 1-2 \'f6rnek STEP dosyas\u305 yla y\'fczey \'e7\u305 kar\u305 m\u305 n\u305  do\u287 rulamak. Joint s\u305 n\u305 fland\u305 rma mant\u305 \u287 \u305 na (en riskli k\u305 s\u305 m) ancak IR sa\u287 lamla\u351 t\u305 ktan sonra ge\'e7mek isabetli olur.\
Nas\uc0\u305 l ilerlemek istersin \'97 elinde test edebilece\u287 imiz \'f6rnek bir STEP/FCStd dosyas\u305  var m\u305 , yoksa \'f6nce IR \u351 emas\u305 n\u305  ve A\u351 ama 0-2'yi mi kodlayal\u305 m?\
\pard\pardeftab720\sa321\partightenfactor0

\f0\b\fs48 \cf0 Geli\uc0\u351 tirme Ortam\u305  Mimarisi: Test Harness + Godot Debug/Edit Katman\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \'d6ncekiyle ayn\uc0\u305  disiplinde ilerliyorum: \'f6nce mimari, kod yok. Burada iki ayr\u305  ama s\u305 k\u305  ba\u287 l\u305  sistem tasarl\u305 yoruz \'97 
\f0\b Python taraf\uc0\u305  geli\u351 tirme aray\'fcz\'fc
\f1\b0  ve 
\f0\b Godot taraf\uc0\u305  debug/edit ortam\u305 
\f1\b0 . \uc0\u304 kisini birbirine ba\u287 layan tek \u351 ey IR/export JSON'\u305  olmal\u305 ; hi\'e7biri di\u287 erinin i\'e7 yap\u305 s\u305 n\u305  bilmemeli.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 0. Temel Mimari Karar: "Generated vs Override" Ayr\uc0\u305 m\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Bu, senin en kritik gereksinimini (hem otomatik hem d\'fczenlenebilir olmak) \'e7\'f6zen tek karard\uc0\u305 r, o y\'fczden en ba\u351 ta netle\u351 tiriyorum:\
Pipeline her \'e7al\uc0\u305 \u351 t\u305 \u287 \u305 nda \'fcretti\u287 i veri 
\f0\b her zaman yeniden \'fcretilebilir ve s\uc0\u305 f\u305 rdan overwrite edilebilir
\f1\b0  olmal\uc0\u305 . Kullan\u305 c\u305 n\u305 n Godot'ta yapt\u305 \u287 \u305  manuel d\'fczeltmeler ise 
\f0\b ayr\uc0\u305  bir katmanda
\f1\b0  saklanmal\uc0\u305 . \u304 kisi asla ayn\u305  dosyada kar\u305 \u351 mamal\u305 . Aksi halde "pipeline'\u305  tekrar \'e7al\u305 \u351 t\u305 rd\u305 m, elle d\'fczeltti\u287 im \u351 ey silindi" ya da tam tersi "pipeline'\u305  g\'fcncelledim ama eski elle-d\'fczeltilmi\u351  veri h\'e2l\'e2 duruyor, hangisi do\u287 ru bilmiyorum" durumuna d\'fc\u351 ersin \'97 bu t\'fcr sistemlerde en yayg\u305 n g\'fcven kayb\u305  sebebi budur.\
Somut olarak \'fc\'e7 katman:\
\pard\pardeftab720\partightenfactor0

\f2\fs26 \cf0 robot_generated.json   \uc0\u8592  pipeline \'e7\u305 kt\u305 s\u305 , her run'da tamamen overwrite edilir (kaynak: Python)\
robot_overrides.json   \uc0\u8592  kullan\u305 c\u305 n\u305 n Godot'ta yapt\u305 \u287 \u305  de\u287 i\u351 iklikler, pipeline dokunmaz (kaynak: Godot)\
robot_final.tscn       \uc0\u8592  ikisinin merge edilmi\u351  hali, Godot'un ger\'e7ekten a\'e7t\u305 \u287 \u305  sahne\
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Merge kural\uc0\u305  basit: override varsa override kazan\u305 r, yoksa generated kullan\u305 l\u305 r, her override alan\u305  "bu neden override edildi" notuyla birlikte tutulur (kim, ne zaman, hangi joint). B\'f6ylece hem "otomatik sistem as\u305 l \'fcr\'fcn" ilkesi korunur hem de manuel m\'fcdahale kaybolmaz.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 1. Python Taraf\uc0\u305 : Test Harness\
\pard\pardeftab720\sa280\partightenfactor0

\fs28 \cf0 \strokec2 1a. Neden web tabanl\uc0\u305  aray\'fcz (Textual/Tkinter de\u287 il)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \strokec2 Senin ihtiya\'e7lar\uc0\u305 n \'97 a\u351 ama g\'f6rselle\u351 tirme, zaman \'e7izelgesi, cache durumu, hata detay\u305 , tekrar tekrar h\u305 zl\u305  deneme \'97 bunlar\u305 n hepsi 
\f0\b durum g\'f6rselle\uc0\u351 tirme + tek tu\u351  tetikleme
\f1\b0  i\uc0\u351 i, karma\u351 \u305 k bir GUI de\u287 il. Bunun i\'e7in a\u287 \u305 r bir masa\'fcst\'fc uygulamas\u305  yerine:\
\pard\tx220\tx720\pardeftab720\li720\fi-720\partightenfactor0
\ls4\ilvl0
\f0\b \cf0 \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Backend:
\f1\b0  Pipeline zaten Python; FastAPI ile ince bir servis katman\uc0\u305  \'97 pipeline'\u305  \'e7al\u305 \u351 t\u305 r\u305 r, a\u351 ama durumlar\u305 n\u305  stream eder (Server-Sent Events / WebSocket).\
\ls4\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Frontend:
\f1\b0  Tek sayfa, framework's\'fcz veya minimal (htmx benzeri) HTML \'97 \'e7\'fcnk\'fc bu senin "nihai \'fcr\'fcn" de\uc0\u287 il, geli\u351 tirme arac\u305 . Karma\u351 \u305 k bir SPA'ya yat\u305 r\u305 m yapmaya de\u287 mez.\
\pard\pardeftab720\sa240\partightenfactor0
\cf0 \strokec2 Bu ayr\uc0\u305 m\u305 n as\u305 l sebebi: backend'i CLI'dan da, gelecekte ba\u351 ka bir aray\'fczden de tetikleyebilmen. Test harness UI, pipeline'\u305 n 
\f3\i tek
\f1\i0  aray\'fcz\'fc olmamal\uc0\u305  \'97 pipeline'\u305 n kendisi bir k\'fct\'fcphane, harness sadece onu izlemek i\'e7in ince bir kabuk.\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 \strokec2 1b. Aray\'fcz d\'fczeni\
\pard\pardeftab720\partightenfactor0

\f2\b0\fs26 \cf0 \strokec2 \uc0\u9484 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9488 \
\uc0\u9474  [ STEP dosyas\u305  se\'e7 ]  [ \u9654  Pipeline'\u305  \'c7al\u305 \u351 t\u305 r ]      \u9474 \
\uc0\u9500 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9508 \
\uc0\u9474  A\u351 ama Durumu:                                        \u9474 \
\uc0\u9474   \u10003  Import & Normalize        cached    12ms          \u9474 \
\uc0\u9474   \u10003  Part Segmentation         cached    8ms            \u9474 \
\uc0\u9474   \u10227  Surface Feature Extraction running  1.2s            \u9474 \
\uc0\u9474   \u9675  Contact Graph              beklemede                \u9474 \
\uc0\u9474   \u9675  Joint Classification       beklemede                \u9474 \
\uc0\u9474   \u9675  Kinematic Tree              beklemede                \u9474 \
\uc0\u9474   \u9675  Pivot & Axis Resolution     beklemede                \u9474 \
\uc0\u9474   \u9675  Validation                  beklemede                \u9474 \
\uc0\u9474   \u9675  Godot Export                beklemede                \u9474 \
\uc0\u9500 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9508 \
\uc0\u9474  [ Godot'a Aktar ]   [ Godot'ta A\'e7 ]                   \u9474 \
\uc0\u9500 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9508 \
\uc0\u9474  Sonu\'e7 \'d6zeti: 6 link, 6 joint (5 revolute, 1 fixed)    \u9474 \
\uc0\u9474  \u9888  1 joint d\'fc\u351 \'fck confidence (0.58) \'97 detay i\'e7in t\u305 kla   \u9474 \
\uc0\u9492 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9496 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Her a\uc0\u351 ama sat\u305 r\u305  \'fc\'e7 \u351 eyi net g\'f6stermeli: 
\f0\b durum ikonu
\f1\b0  (cache/running/done/failed), 
\f0\b s\'fcre
\f1\b0 , ve t\uc0\u305 klan\u305 nca a\'e7\u305 lan 
\f0\b detay paneli
\f1\b0  (o a\uc0\u351 aman\u305 n girdi/\'e7\u305 kt\u305  \'f6zeti + hata varsa tam stack trace + hangi veri neden reddedildi). Hata durumunda harness asla sadece "pipeline failed" demez \'97 hangi a\u351 amada, hangi par\'e7a/joint \'fczerinde, hangi varsay\u305 m\u305 n k\u305 r\u305 ld\u305 \u287 \u305  g\'f6sterilir. Bu senin "neden ba\u351 ar\u305 s\u305 z oldu\u287 unu anlayabilmeliyim" gereksinimin.\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 \strokec2 1c. Cache g\'f6r\'fcn\'fcrl\'fc\uc0\u287 \'fc \'97 mimarinin do\u287 al sonucu\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \strokec2 \'d6nceki mesajda tan\uc0\u305 mlad\u305 \u287 \u305 m\u305 z content-addressed cache (her a\u351 ama 
\f2\fs26 hash(girdi + parametreler + versiyon)
\f1\fs24  ile diske yaz\uc0\u305 l\u305 yor) burada do\u287 rudan i\u351 ine yar\u305 yor: harness her a\u351 ama i\'e7in cache'te bir isabet olup olmad\u305 \u287 \u305 n\u305  zaten biliyor, \'e7\'fcnk\'fc hash'i hesaplay\u305 p bak\u305 yor. Yani "hangi a\u351 ama cache'ten geldi" sorusu ayr\u305  bir mekanizma gerektirmiyor \'97 pipeline'\u305 n kendi i\'e7 i\u351 leyi\u351 inin bir yan \'fcr\'fcn\'fc. Harness bunu sadece g\'f6r\'fcn\'fcr k\u305 l\u305 yor.\
Pratik sonu\'e7: ayn\uc0\u305  STEP dosyas\u305  \'fczerinde yaln\u305 zca joint-classification e\u351 i\u287 ini de\u287 i\u351 tirip tekrar \'e7al\u305 \u351 t\u305 rd\u305 \u287 \u305 nda, Import/Segmentation/Surface-Extraction a\u351 amalar\u305  (en pahal\u305  olanlar, saniyeler s\'fcrebilir) an\u305 nda cache'ten d\'f6ner, sadece A\u351 ama 5'ten itibaren yeniden hesaplan\u305 r. Bu senin "h\u305 zl\u305  iterasyon" beklentini do\u287 rudan kar\u351 \u305 l\u305 yor.\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 \strokec2 1d. DecisionTrace g\'f6r\'fcn\'fcm\'fc\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \strokec2 Her joint i\'e7in, A\uc0\u351 ama 8'de \'fcretti\u287 imiz 
\f2\fs26 evidence[]
\f1\fs24  listesi harness'ta ayr\uc0\u305  bir panelde g\'f6sterilmeli \'97 a\u287 a\'e7 yap\u305 s\u305 nda de\u287 il, d\'fcz okunabilir liste:\
\pard\pardeftab720\partightenfactor0

\f2\fs26 \cf0 Joint J3 (link_forearm \uc0\u8594  link_upper_arm)\
  Tip: revolute   Confidence: 0.91\
  Kan\uc0\u305 t:\
   - E\uc0\u351  eksenli silindirik y\'fczey \'e7ifti bulundu (r=14.2mm, sapma 0.03mm)\
   - Eksene dik iki flan\uc0\u351  temas\u305  \u8594  eksenel \'f6teleme engellendi\
   - Aday eksenler: 1 (tekil, belirsizlik yok)\
  Reddedilen adaylar: yok\
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Bu g\'f6r\'fcn\'fcm hem harness'ta hem Godot'ta 
\f0\b ayn\uc0\u305  JSON kayna\u287 \u305 ndan
\f1\b0  okunmal\uc0\u305  \'97 iki ayr\u305  yerde iki farkl\u305  implementasyon istemiyoruz.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 2. Python \uc0\u8596  Godot K\'f6pr\'fcs\'fc\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 "Tek t\uc0\u305 kla Godot'ta test" iste\u287 i i\'e7in iki se\'e7enek var, aralar\u305 ndaki fark\u305  netle\u351 tireyim:\
\pard\pardeftab720\sa240\partightenfactor0

\f0\b \cf0 Se\'e7enek A \'97 Dosya tabanl\uc0\u305  (\'f6neri):
\f1\b0  Harness, 
\f2\fs26 robot_generated.json
\f1\fs24  ve mesh dosyalar\uc0\u305 n\u305  proje klas\'f6r\'fcne yazar; Godot taraf\u305 nda bir EditorPlugin bu klas\'f6r\'fc izler (FileSystemDock de\u287 i\u351 iklik sinyali) ve "Yeniden Y\'fckle" butonuyla ya da otomatik olarak sahneyi yeniden kurar. Godot'u ayr\u305 ca \'e7al\u305 \u351 t\u305 rmana gerek kalmaz, zaten a\'e7\u305 ksa an\u305 nda g\'fcncellenir.\

\f0\b Se\'e7enek B \'97 Canl\uc0\u305  ba\u287 lant\u305  (soket/RPC):
\f1\b0  Python ve Godot aras\uc0\u305 nda s\'fcrekli a\'e7\u305 k bir ba\u287 lant\u305 . Daha "anl\u305 k" ama gereksiz karma\u351 \u305 kl\u305 k \'97 senin senaryonda (STEP de\u287 i\u351 tir \u8594  analiz et \u8594  Godot'ta bak) dosya tabanl\u305  yakla\u351 \u305 m hem daha basit hem daha g\'fcvenilir (Godot \'e7\'f6kerse Python etkilenmez, tersi de ge\'e7erli).\
A'y\uc0\u305  \'f6neriyorum: basitlik, hata izolasyonu ve "IR tek ger\'e7ek kaynak" ilkesiyle tam uyumlu.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 3. Godot Taraf\uc0\u305 : Debug/Edit Ortam\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Bu bir oyun sahnesi de\uc0\u287 il, bir 
\f0\b EditorPlugin
\f1\b0  (Godot edit\'f6r eklentisi) olmal\uc0\u305  \'97 \'e7\'fcnk\'fc istedi\u287 in \u351 eyler (gizmo \'e7izimi, dock panel, se\'e7im, joint s\'fcr\'fckleme) edit\'f6r zaman\u305  ara\'e7lar\u305 , runtime oyun mekani\u287 i de\u287 il.\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 \strokec2 3a. Sahne/node yap\uc0\u305 s\u305  (senin "temiz ve d\'fczenlenebilir" gereksinimi)\
\pard\pardeftab720\partightenfactor0

\f2\b0\fs26 \cf0 \strokec2 RobotRoot (Node3D)                          \uc0\u8592  robot_final.tscn k\'f6k\'fc\
\uc0\u9500 \u9472 \u9472  RobotMeta (Node)                         \u8592  DecisionTrace + validation verisi (Resource olarak)\
\uc0\u9500 \u9472 \u9472  Link_base (Node3D)\
\uc0\u9474    \u9500 \u9472 \u9472  MeshInstance3D (base geometrisi)\
\uc0\u9474    \u9492 \u9472 \u9472  Link_upper_arm (Node3D)              \u8592  Godot scene tree = kinematik a\u287 a\'e7, birebir\
\uc0\u9474        \u9500 \u9472 \u9472  JointGizmo (Node3D, editor-only)  \u8592  pivot/axis \'e7izimi buradan\
\uc0\u9474        \u9500 \u9472 \u9472  MeshInstance3D\
\uc0\u9474        \u9492 \u9472 \u9472  Link_forearm (Node3D)\
\uc0\u9474            \u9492 \u9472 \u9472  ...\
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Kritik karar: 
\f0\b parent-child ili\uc0\u351 kisi do\u287 rudan Godot scene tree hiyerar\u351 isiyle temsil edilir
\f1\b0 , ayr\uc0\u305  bir "joint graph" veri yap\u305 s\u305  olarak de\u287 il. Yani 
\f2\fs26 Link_forearm
\f1\fs24 , 
\f2\fs26 Link_upper_arm
\f1\fs24 '\uc0\u305 n ger\'e7ek Godot child node'udur. Bunun sebebi: Godot'un transform sistemi zaten parent-relative \'e7al\u305 \u351 \u305 yor, bunu joint hiyerar\u351 isiyle e\u351 le\u351 tirirsek "joint'i d\'f6nd\'fcr \u8594  child otomatik do\u287 ru yerde d\'f6ner" davran\u305 \u351 \u305 n\u305  Godot'un kendi transform sisteminden bedavaya al\u305 r\u305 z, ayr\u305  bir forward-kinematics hesaplamas\u305 na gerek kalmaz.\
Her 
\f2\fs26 Link_X
\f1\fs24  node'u \'fczerine 
\f0\b custom Resource
\f1\b0  (
\f2\fs26 JointData.gd
\f1\fs24  gibi bir 
\f2\fs26 Resource
\f1\fs24  s\uc0\u305 n\u305 f\u305 ) eklenir \'97 pivot, axis, tip, limit, confidence, evidence hepsi bu Resource'ta, Inspector panelinde g\'f6r\'fclebilir ve d\'fczenlenebilir. Bu, "Godot Inspector'da incelenebilir olmal\u305 " gereksinimini native Godot mekanizmas\u305 yla \'e7\'f6zer, \'f6zel bir UI yazmaya gerek kalmaz.\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 \strokec2 3b. Debug Dock (editor paneli)\
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \strokec2 Ayr\uc0\u305  bir alt panel (Godot'un "Bottom Panel" mekanizmas\u305 yla, t\u305 pk\u305  Output/Debugger sekmeleri gibi):\
\pard\pardeftab720\partightenfactor0

\f2\fs26 \cf0 \uc0\u9484 \u9472  Robot Debugger \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9488 \
\uc0\u9474  Link/Joint Listesi:      \u9474   Se\'e7ili Joint: J3   \u9474 \
\uc0\u9474   \u9656  base                  \u9474   Tip: revolute       \u9474 \
\uc0\u9474   \u9662  upper_arm             \u9474   Pivot: (0.12, 0.34, \u9474 \
\uc0\u9474     \u9656  J1 (revolute)       \u9474          0.08)         \u9474 \
\uc0\u9474   \u9662  forearm                \u9474   Axis: (0,1,0)        \u9474 \
\uc0\u9474     \u9656  J2 (revolute)       \u9474   Confidence: 0.91     \u9474 \
\uc0\u9474   \u9662  wrist                  \u9474   [Axis'i g\'f6ster \u10003 ]    \u9474 \
\uc0\u9474     \u9656  J3 (revolute) \u9664 sel  \u9474   [Test slider: -45\'b0..+45\'b0] \u9474 \
\uc0\u9474                             \u9474   [\u9654  T\'fcm joint'leri s\u305 rayla test et] \u9474 \
\uc0\u9474                             \u9474   [Evidence detay\u305  \u9662 ]  \u9474 \
\uc0\u9492 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9496 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\fs24 \cf0 Fonksiyonlar:\
\pard\tx220\tx720\pardeftab720\li720\fi-720\partightenfactor0
\ls5\ilvl0
\f0\b \cf0 \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Se\'e7im \uc0\u8594  gizmo:
\f1\b0  bir joint se\'e7ildi\uc0\u287 inde 3D viewport'ta pivot noktas\u305  (k\'fcre) ve eksen (ok, belirgin renk) \'e7izilir \'97 bu bir 
\f2\fs26 EditorNode3DGizmoPlugin
\f1\fs24  ile yap\uc0\u305 l\u305 r, kal\u305 c\u305  sahne node'u olarak de\u287 il (editor-only overlay).\
\ls5\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Parent/child renklendirme:
\f1\b0  se\'e7ili joint'in child alt-a\uc0\u287 ac\u305 ndaki t\'fcm mesh'ler bir highlight rengiyle (\'f6rn. yar\u305  saydam turuncu overlay material) vurgulan\u305 r, parent taraf\u305  n\'f6tr kal\u305 r \'97 child'\u305 n ger\'e7ekten do\u287 ru alt-a\u287 a\'e7 olup olmad\u305 \u287 \u305 n\u305  g\'f6zle do\u287 rulamak i\'e7in.\
\ls5\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Test slider:
\f1\b0  se\'e7ili joint'in local rotation/translation'\uc0\u305 n\u305  canl\u305  de\u287 i\u351 tirir (sadece editor'de, 
\f2\fs26 Link_X.rotation
\f1\fs24  \'fczerinden) \'97 child node zaten Godot transform hiyerar\uc0\u351 isi sayesinde do\u287 ru pivot etraf\u305 nda d\'f6ner, ekstra hesaplama gerekmez.\
\ls5\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 S\uc0\u305 rayla t\'fcm joint'leri test et:
\f1\b0  her joint'i k\uc0\u305 sa bir aral\u305 kta otomatik salla (basit bir tween d\'f6ng\'fcs\'fc), b\'f6ylece tek tek t\u305 klamadan hepsini g\'f6zden ge\'e7irebilirsin.\
\ls5\ilvl0
\f0\b \kerning1\expnd0\expndtw0 \outl0\strokewidth0 {\listtext	\uc0\u8226 	}\expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 Evidence paneli:
\f1\b0  
\f2\fs26 RobotMeta
\f1\fs24  Resource'undan okunan DecisionTrace'i d\'fcz metin olarak g\'f6sterir (Python harness'taki ayn\uc0\u305  g\'f6r\'fcn\'fcm, ayn\u305  JSON kayna\u287 \u305 ndan).\
\pard\pardeftab720\sa280\partightenfactor0

\f0\b\fs28 \cf0 3c. Manuel d\'fczenleme \uc0\u8594  override kayd\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 \strokec2 Kullan\uc0\u305 c\u305  Inspector'da bir 
\f2\fs26 JointData
\f1\fs24  Resource'unun pivot/axis alan\uc0\u305 n\u305  elle de\u287 i\u351 tirdi\u287 inde, bu de\u287 i\u351 iklik do\u287 rudan 
\f2\fs26 .tscn
\f1\fs24 'e yaz\uc0\u305 lmaz \'97 plugin bunu yakalar (
\f2\fs26 property_changed
\f1\fs24  sinyali ya da "De\uc0\u287 i\u351 iklikleri Kaydet" butonu) ve 
\f2\fs26 robot_overrides.json
\f1\fs24 'a, hangi joint'in hangi alan\uc0\u305 n\u305 n kim taraf\u305 ndan/ne zaman override edildi\u287 i bilgisiyle yazar. Bir sonraki pipeline \'e7al\u305 \u351 t\u305 rmas\u305 nda bu override korunur; harness taraf\u305 nda da "3 joint'te aktif override var" \u351 eklinde g\'f6r\'fcn\'fcr olur \'97 b\'f6ylece hangi verinin otomatik hangi verinin elle d\'fczeltildi\u287 i hi\'e7bir zaman belirsizle\u351 mez.\
\pard\pardeftab720\sa298\partightenfactor0

\f0\b\fs36 \cf0 4. Mesh'in Ayr\uc0\u305 \u351 t\u305 r\u305 lmas\u305 \
\pard\pardeftab720\sa240\partightenfactor0

\f1\b0\fs24 \cf0 Senin belirtti\uc0\u287 in gibi mesh \'fcretimi kinematik do\u287 ruluktan ba\u287 \u305 ms\u305 z bir katman olmal\u305 . Bunu harness'ta somutla\u351 t\u305 r\u305 yorum: pipeline'\u305 n A\u351 ama 0-8'i (kinematik \'e7\u305 kar\u305 m) her zaman \'e7al\u305 \u351 \u305 r ve h\u305 zl\u305 d\u305 r (mesh yok, sadece B-Rep analiz + hafif geometri). Mesh export (A\u351 ama 10, STEP geometrisinden glTF tesselasyonu) 
\f0\b ayr\uc0\u305  bir buton/tetikleyici
\f1\b0  ile iste\uc0\u287 e ba\u287 l\u305  \'e7al\u305 \u351 \u305 r \'97 harness'ta "Kinematik Analiz Et" ve "Mesh \'dcret (CAD kalitesinde)" ayr\u305  aksiyonlar olarak durur. Godot debug ortam\u305 nda mesh olmadan da (basit primitive/wireframe placeholder ile) pivot/axis/hiyerar\u351 i do\u287 rulamas\u305  yap\u305 labilmeli \'97 \'e7\'fcnk\'fc senin as\u305 l test d\'f6ng\'fcn "pivot do\u287 ru mu" sorusu, bu soru i\'e7in CAD-kalite mesh beklemek gereksiz.\
\pard\pardeftab720\partightenfactor0
\cf3 \strokec3 \
\pard\pardeftab720\sa240\partightenfactor0
\cf0 \strokec2 Bu mimariyle \'f6nerim: \'f6nce 
\f0\b IR \uc0\u351 emas\u305  + A\u351 ama 0-2'yi
\f1\b0  senin y\'fckledi\uc0\u287 in 
\f2\fs26 6_AXIS_robot_arm.step
\f1\fs24  \'fczerinde \'e7al\uc0\u305 \u351 \u305 r hale getirip, harness'\u305 n en iskelet halini (dosya se\'e7 \u8594  \'e7al\u305 \u351 t\u305 r \u8594  a\u351 ama durumu g\'f6ster) kural\u305 m. Godot plugin'ine, kinematik \'e7\u305 kar\u305 m en az bir robotta g\'fcvenilir sonu\'e7 verdikten sonra ge\'e7mek daha isabetli \'97 aksi halde hen\'fcz stabil olmayan bir veri format\u305 na g\'f6re Godot taraf\u305  in\u351 a edip iki kez yazm\u305 \u351  oluruz.\
Onaylarsan bu s\uc0\u305 rayla kodlamaya ba\u351 layabiliriz: 
\f0\b (1) IR veri s\uc0\u305 n\u305 flar\u305  \u8594  (2) STEP import + par\'e7a segmentasyonu \u8594  (3) y\'fckledi\u287 in dosya \'fczerinde ilk \'e7\u305 kt\u305  \u8594  (4) harness iskeleti.
\f1\b0  Nas\uc0\u305 l ilerleyelim?\
}