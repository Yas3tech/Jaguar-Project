# Jaguar-Project: uitgebreide uitleg

De korte versie staat in de [README](../README.md). Hier staat hoe elk deel werkt, met de getallen uit de code. Elke map heeft daarnaast een eigen README.

**Inhoud**

1. [Het robot en de verbinding](#1-het-robot-en-de-verbinding)
2. [Waar is het robot? De odometrie](#2-waar-is-het-robot-de-odometrie)
3. [Rechtdoor rijden](#3-rechtdoor-rijden)
4. [Draaien ter plaatse](#4-draaien-ter-plaatse)
5. [Een parcours](#5-een-parcours)
6. [Een pad volgen in één beweging](#6-een-pad-volgen-in-één-beweging)
7. [Naar GPS-coördinaten](#7-naar-gps-coördinaten)
8. [De camera en obstakels](#8-de-camera-en-obstakels)
9. [Veiligheid en batterij](#9-veiligheid-en-batterij)
10. [Bediening vanuit de browser](#10-bediening-vanuit-de-browser)
11. [Live meekijken en Grafana](#11-live-meekijken-en-grafana)
12. [Opslaan, analyse en dashboard](#12-opslaan-analyse-en-dashboard)
13. [Kalibreren](#13-kalibreren)
14. [Zonder robot: simulatoren en tests](#14-zonder-robot-simulatoren-en-tests)
15. [Bestanden in config/](#15-bestanden-in-config)

---

## 1. Het robot en de verbinding

| Onderdeel | Gegevens |
|---|---|
| Robot | Dr Robot Jaguar 4x4, skid-steer (geen stuur: links en rechts verschillend snel), 61,5 × 57,3 cm |
| Wifi | `DriJaguar` (adressen en login in `docs/robot/Notes.txt`) |
| Besturing | TCP `192.168.0.60:10001`, tekstregels die eindigen op CRLF |
| Motordrivers | twee Roboteq-borden: `MM0` (voorwielen) en `MM1` (achterwielen) |
| IMU | gyro en accelerometer, 50 Hz |
| GPS | Garmin GPS 18x-5Hz, 5 posities per seconde |
| Camera | Axis, `192.168.0.65:8081`, MJPEG |

**Wat het robot stuurt**

| Regel | Hoe vaak | Inhoud |
|---|---|---|
| `#nr,YAW,..,GYRO,x,y,z,ACC,x,y,z,..` | 50 Hz | ruwe gyro en accelerometer (counts) met een volgnummer |
| `MM0 C=links:rechts`, `MM1 C=..` | 5 Hz (vaste klok van 0,2 s) | encoderstand van de 4 wielen |
| `MM0 V=..` | 5 Hz | spanningen, waaronder de batterij |
| `$GPRMC,..` | 5 Hz | breedte, lengte, snelheid en koers over de grond, fix |
| `MM0 FF=..` | 1 Hz | foutvlaggen, waaronder de E-Stop |

**Wat de laptop stuurt** ([kern/robot.py](../jaguar/kern/robot.py))

- elke 100 ms `PING` en `MMW !M <links> <-rechts>`: open-loop vermogen, -1000..1000. Komt er geen PING meer, dan zet de controller de motoren uit (watchdog);
- `MMW !MG` geeft de E-Stop vrij bij de start, `MMW !EX` zet hem terug aan na elke rit.

De drivers laten soms een commando vallen of plakken twee regels aan elkaar ("`!P 2 -474PING`"). Daarom wacht elk drivercommando op zijn bevestiging en wordt het herhaald.

**De rustfase.** Elke rit begint met 3 s stilstaan: het script meet dan de gyrobias en de zwaartekrachtrichting. Beweegt het robot in die tijd, dan meet het opnieuw.

**De tijd.** De wifi houdt pakketten soms 0,5 tot 0,7 s vast en levert ze dan samen af. Het script telt de tijd daarom met de volgnummers van de IMU (50 Hz), niet met het moment van ontvangst.

---

## 2. Waar is het robot? De odometrie

[kern/odometrie.py](../jaguar/kern/odometrie.py) rekent afstand, koers en positie uit. Dezelfde code draait live in elk rijscript en achteraf in de analyse.

**Koers:** de gyro (Z-as), min de bias uit de rustfase. Het kompas is onbruikbaar: de motoren storen het.

**Afstand, per encoderpakket (5 Hz):**

1. **Snelheid per wiel** = pulsen ÷ (n × 0,2 s). Met de vaste klok van de controller is dat 3,5× minder ruizig dan met de ontvangsttijd.
2. **Per kant:** voor- en achterwiel zijn het eens (verschil ≤ 0,10 m/s + 5 %) → gemiddelde. Anders telt het wiel dat het dichtst bij de voorspelling van de IMU ligt; het andere slipt.
3. **Links tegen rechts:** het verschil moet passen bij de draaisnelheid van de gyro × de spoorbreedte. Bij skid-steer is het altijd tot 2,5× groter (de wielen schuiven zijwaarts in een bocht). Pas een groter verschil is een slippende kant.
4. **Kalman-filter** op [snelheid, versnellingsbias]: de accelerometer voorspelt (met de helling eraf, uit de gyro-Y), de wielen meten. Ligt de wielsnelheid meer dan 0,25 m/s (+ 3σ) van de voorspelling, dan worden de wielen verworpen.
5. **Wielen verworpen:** de IMU-integratie telt, begrensd tussen 0 en het snelste wiel. Dat heet "IMU overbrugt". Na 0,6 s heet het "afstand onzeker"; na 1 s stopt het rijscript.
6. **Stilstand** (geen pulsen, trilling < 0,03 g): snelheid 0, gyrobias en helling worden bijgeleerd.
7. **Wielen in de lucht:** snelle wielen zonder trilling (< 0,04 g) en zonder dat de IMU het robot voelt optrekken, of meer dan 60° gekanteld.

**Positie:** x += ds · cos(koers), y += ds · sin(koers). x = vooruit, y = links, vanaf de start.

**Waarop dat gebaseerd is** (ritten van 2 en 8 oktober):

- voor- en achterwiel hebben in schone stukken dezelfde snelheid (verschil < 0,04 m/s bij p90);
- de IMU alleen is na 0,2 s 1,5 cm fout, na 0,6 s 7 cm, na 1 s 17 cm, na 2 s 70 cm: goed om kort te overbruggen, niet om afstand te meten;
- wielen die de grond niet raken trillen ongeveer 12× minder (0,035 g tegen 0,43 g);
- rit `134323`: na een botsing kantelde het robot op zijn neus en telden de encoders nog 7 m terwijl het stilstond. De odometrie hield daarvan −0,17 m over.

**Grens:** slippen alle vier de wielen evenveel aan constante snelheid (gras, zand), dan zijn de wielen het eens en meet de accelerometer niets. In de simulatie gaf 15 % slip over 1,5 m 26 cm te veel. Daarvoor dienen de GPS-schaal (§7) en de proef met vrijlopende meetwielen (§13).

---

## 3. Rechtdoor rijden

```powershell
python -m jaguar.rijden.rechtdoor --power 250 --distance 5
python -m jaguar.rijden.heen_en_terug --power 250 --distance 1.5 --pause 1.5
```

**Eén beweging:** optrekken over `--ramp` seconden (standaard 1,5), rijden, afremmen, en het vermogen uit zetten op het moment dat het uitrollen het robot precies op het doel brengt. Het remvermogen volgt √(2 · rem · resterende weg) ÷ (snelheid per vermogen).

- **Uitloop:** hoe ver het robot nog rolt na vermogen 0. Wordt na elke stop bijgeleerd (30 % oud, 70 % nieuw).
- **Te kort** (meer dan `--tolerance`, standaard 10 mm): hoogstens twee keer een stukje vooruit.
- **Te ver:** alleen een overschot van meer dan 10 cm wordt teruggereden.
- **`--precies`:** stopt 2 cm voor het doel en doet de rest met kruipstoten (`--creep`, standaard 60). Op het echte robot regelt de driver de laatste 30 cm zelf in positiemodus (Roboteq `MMOD 3`, PID 255/0/120, 40 % vermogen); gemeten: 1 m en 5 m binnen 1 à 3 mm volgens de encoders. Daarna terug naar de fabrieksinstelling (open loop, PID 30/10/0). Uitzetten met `JAGUAR_DRIVER=0`.

**Rechtdoor blijven (cascade-PID)**

- Buiten: de zijdelingse afwijking geeft een gewenste koers, 20° per meter, hoogstens 5°, terug naar de startlijn.
- Binnen: een PID op de gyrokoers geeft het vermogensverschil links/rechts. P = 6 per graad, I = 4 per graad·s (een kant die zwakker trekt; gespiegeld bij achteruit), D = 0,4 per graad/s (op de gemeten draaisnelheid, dempt slingeren).
- Het verschil is begrensd op 30 % van het rijvermogen; de I stopt met groeien zolang dat zo is (anti-windup).
- Zonder regeling draaide het robot bij het optrekken 3,6° weg en stond het na 3 m 17 cm naast de lijn.

**Eindkoers:** wijkt de koers aan het eind meer dan 1,5° af, dan draait het robot die recht.

**Richting:** `--distance -3` of `--power -250` is achteruit. `--duration` rijdt op tijd zoals vroeger.

---

## 4. Draaien ter plaatse

- Op de gyro: het vermogensverschil zakt mee met de resterende hoek, en het robot stopt vroeger volgens zijn draaisnelheid (draai-uitloop, bijgeleerd).
- Daarna corrigeert het met korte stoten, alleen als de fout groter is dan 1° (0,4° met `--precies`). Het leert hoeveel graden een stoot geeft en stopt als de kleinste stoot de fout alleen groter zou maken.
- Komt het niet in beweging, dan verhoogt het het minimumvermogen stap voor stap.
- Na elke draai vergelijkt het gyro en encoders (`RATIO` in de CSV) en meldt het een plotse verandering van meer dan 15 %.
- Zo juist als de gyroschaal: kalibreren met `test_sondewiel draai` (§13).

---

## 5. Een parcours

```powershell
python -m jaguar.rijden.parcours vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5
python -m jaguar.rijden.parcours v:2 l:90 v:2 l:90 v:2 l:90 v:2
```

- Elke rechte stap rijdt naar het **geplande hoekpunt**, gerekend vanaf de start, en volgt de geplande lijn. Een fout van de vorige stap (te ver, te kort, naast de lijn) werkt de volgende vanzelf weg, tot 50 cm per stap.
- Bogen uit de ontwerper rijden "in stukjes" (rechte stukjes van 10° met draaien ertussen) of "vloeiend" (`Jaguar.arc()`: straal minstens 0,5 m, vermogen hoogstens 220).
- Stukken die vloeiend op elkaar aansluiten worden samen één pad (§6), zonder stoppen.
- Wat het parcours doet bij een obstakel: zie §8.

---

## 6. Een pad volgen in één beweging

`Jaguar.volg(pad, vermogen)` in [kern/robot.py](../jaguar/kern/robot.py) rijdt een lijst punten (om de 2 cm) zonder te stoppen.

**Elke 0,05 s:**

1. **Waar op het pad:** het robot schuift op naar het dichtste punt (alleen vooruit langs het pad).
2. **Hoe ver ernaast:** de afstand loodrecht op het pad, + = links. Meer dan 1 m: stoppen, er klopt iets niet.
3. **Doelkoers** = richting van het pad op 0,15 s rijden verderop (`PAD_RICHTING_S`) + 100° per meter naast het pad (`PAD_K_NAAST`), hoogstens 25° (`PAD_MAX_BIJ`).
4. **Voorspelde stuurwaarde:** de kromming van het pad tussen hier en 0,3 m + 0,4 s rijden verderop bepaalt hoeveel sneller de buitenste wielen draaien (hoogstens 90 % verschil).
5. **De gyro-PID** stuurt de rest bij, hoogstens 35 % van het vermogen extra.
6. **Snelheid:** optrekken over `ramp`; 1,5 s voor een bocht (kromming boven 0,05 per meter, straal kleiner dan 20 m) al terug naar hoogstens 220; afremmen naar het laatste punt zoals bij rechtdoor. Het vermogen verandert geleidelijk, nooit in sprongen.

**Waarom deze waarden** (ε-baan van 42 m, echte positie in `sim_robot`):

| `PAD_K_NAAST` / `PAD_MAX_BIJ` / `PAD_RICHTING_S` | Gemiddeld naast de lijn | Hoogstens |
|---|---|---|
| 40 / 15 / 0,3 (vroeger) | 9,9 cm | 25,0 cm |
| 70 / 25 / 0,2 | 5,0 cm | 14,7 cm |
| **100 / 25 / 0,15 (nu)** | **3,9 cm** | **11,6 cm** |
| 100 / 25 / 0,1 | 3,5 cm | 11,8 cm |

Met 0,3 s vooruitkijken sneed het robot elke bocht 5 cm aan de binnenkant af en kwam het op de korte rechte stukken niet meer terug op de lijn. 0,1 s is nog iets beter met snelle motoren, maar gaat slingeren met tragere motoren. Slingert het robot na een bocht: terug naar 70 en 0,2.

---

## 7. Naar GPS-coördinaten

```powershell
python -m jaguar.rijden.gps_route 50.8400000,4.3200000 50.8400449,4.3200000 --richting 0
python -m jaguar.rijden.gps_route --punten-bestand route.txt --richting 0 --vloeiend
python -m jaguar.rijden.gps_route --vanaf-hier 50.84010,4.32000
python -m jaguar.rijden.gps_route <punten> --richting 0 --droog          # alleen het plan
```

**Wat de GPS kan:** fout kleiner dan 15 m in 95 % van de gevallen, met WAAS/EGNOS kleiner dan 3 m, alleen onder open hemel. Hij geeft alleen `GPRMC` door, zonder satellieten of HDOP. Binnen meldt hij toch een fix terwijl de positie 20 tot 50 m dwaalt. De precisie komt dus van de wielen en de gyro; de GPS levert coördinaten in de wereld en het noorden.

**Van coördinaten naar rijden** ([navigatie/gps.py](../jaguar/navigatie/gps.py)):

1. **Lezen:** `parse_rmc` controleert de controlesom en geeft breedte, lengte, snelheid, koers en fix.
2. **Naar meters:** `Lokaal` legt een vlak (oost, noord) in meter rond het eerste punt, met de WGS84-formules voor meter per graad. Op een paar honderd meter is de fout kleiner dan 1 cm.
3. **Waar het robot begint:** standaard staat het robot op het eerste punt (het nauwkeurigst). Met `--vanaf-hier` is de start het gemiddelde van de GPS in rust (`--gps-wacht`, standaard 10 s, minstens 10 posities). Springt de GPS dan meer dan 8 m, dan weigert het script.
4. **Het noorden:** `--richting` (kompaskoers, 0 = noord, 90 = oost). Zonder `--richting` rijdt het robot eerst 6 m rechtdoor op vermogen 400 (ongeveer 1,1 m/s; daaronder is de koers over de grond onbruikbaar) en haalt het noorden uit (a) de koers over de grond en (b) de vorm van het gereden spoor. De nauwkeurigste telt, als die beter is dan 5°.
5. **Naar het robotstelsel:** `enu_naar_robot` draait (oost, noord) met het noorden naar (vooruit, links).
6. **Rijden:** punt per punt (draaien en rechtdoor, §4 en §5), of met `--vloeiend` als één afgerond pad (twee keer Chaikin, punt om de 2 cm) dat `volg` rijdt (§6).

**De GPS onderweg** (`--gps auto`, standaard):

- **Uitlijning:** het gereden spoor wordt op het GPS-spoor gelegd (kleinste kwadraten: draaiing, verschuiving, schaal). De draaiing verbetert het noorden, de schaal verraadt slip (echte meters per getelde meter). Een vaste fout van de GPS doet er niet toe: alleen de vorm telt.
- **Vertraging:** de GPS loopt tot 1 s achter. Het script past het spoor voor 0; 0,2; … 1 s vertraging en neemt de beste.
- **De GPS-positie zelf** telt pas als haar fout kleiner is dan die van de wielen (2 % van de gereden afstand + de onzekerheid op het noorden). Met de GPS van het robot pas na tientallen meters.
- **Past het spoor niet** (binnen, tussen gebouwen): de GPS telt niet.
- `--gps uit`: alleen wielen en gyro. `--gps aan`: de GPS-positie altijd zodra het spoor past.
- Met `--vloeiend` ligt het pad vast bij het vertrek; de GPS kijkt alleen mee.

**Na de rit** drukt het script af:

- de positie **volgens wielen en gyro**: de odometrie met het noorden terug omgerekend naar breedte/lengte (`robot_naar_enu` + `Lokaal.naar_ll`), en de afstand tot het laatste punt;
- de positie **volgens de GPS**: het gemiddelde van de laatste 2 s;
- hoe goed het GPS-spoor op het gereden spoor past (rms), het noorden volgens het spoor, de schaal, en per punt wat er gebruikt is.

**GPS meten in de ontwerper:** zet het robot op de plek, klik **GPS meten**. Een minuut lang (E-Stop blijft aan) kijkt de pagina hoe de positie beweegt: **goed** (spreiding ≤ 3 m, wegloop ≤ 2 m), **matig** (≤ 8 m en ≤ 5 m) of **slecht**. Een rustige positie is nog geen juiste positie: weerkaatsing tegen een gebouw geeft een vaste fout van enkele meters.

---

## 8. De camera en obstakels

[camera/obstakel.py](../jaguar/camera/obstakel.py). Standaard uit; aanzetten op de live pagina met **Opties → Camera** (beeld) en **Opties → Obstakels** (stoppen). Nodig: `pip install opencv-python numpy`.

### Het beeld

- Axis-camera op `192.168.0.65:8081`, Digest-login, MJPEG op `/axis-cgi/mjpg/video.cgi`.
- De live pagina krijgt 640×480 via `jaguar/live/view.py` (`/camera`), dat de login doet, zodat de browser geen wachtwoord nodig heeft.
- De detectie vraagt een eigen stroom van **320×240 aan 10 beelden per seconde**: klein, zodat de wifi de robotgegevens niet ophoudt. Een achtergronddraad knipt de JPEG's uit de stroom (`FF D8` … `FF D9`) en verwerkt elk beeld.

### Een muur herkennen

Het idee: de camera staat vast op het robot en kijkt schuin naar de vloer. Waar de vloer ophoudt, begint het dichtste obstakel: de voet van een muur, een deur, een stoelpoot, een been.

Per beeld (`edge_columns`):

1. **Gladmaken en omzetten:** Gauss-filter 7×7 tegen ruis, dan naar de kleurruimte **Lab** (lichtheid + twee kleurassen; een kleurverschil daarin komt overeen met wat een mens ziet).
2. **Elke rij met de vloer eronder vergelijken:** per kolom het gemiddelde van de rijen 3 tot 10 pixels lager (snel, met een cumulatieve som). De vloer mag met de afstand geleidelijk lichter of donkerder worden (reflectie); een plotse sprong niet.
3. **Sprong = rand:** is het Lab-verschil groter dan **15**, dan is dat de rand van de vloer. Ter vergelijking: de vloer zelf verandert hoogstens 7 per 5 rijen, de voet van een muur geeft ongeveer 30.
4. **Van onder naar boven:** per kolom telt de **eerste** sprong vanaf de onderkant (de onderste 3 % van het beeld wordt overgeslagen: daar kan het robot zelf in beeld komen). Dat is het dichtste obstakel in die kolom.
5. **De weg van het robot:** alleen de middelste strook telt, van 38 % tot 62 % van de breedte (ongeveer 0,6 tot 0,8 m breed op 1 à 2 m). Van die kolommen telt het 85e percentiel: de dichtste rand, op een paar ruisende kolommen na.
6. **Ernaast:** links (0–38 %) en rechts (62–100 %) gebeurt hetzelfde met de mediaan, om te weten aan welke kant de vloer vrij is. De pagina krijgt de vloerrand in 32 vakken over de hele breedte.

### Van beeldrij naar afstand

Een vaste camera boven een vlakke vloer: hoe lager in het beeld, hoe dichterbij. Dat volgt

```text
d = k / (y − y_horizon)        y = rij (0 = boven, 1 = onder)
```

- **Kalibreren** (Opties → Obstakels → Kalibreren…): zet het robot voor een muur op een gekende afstand en klik de voet van de muur aan. Minstens twee afstanden. De punten staan in `config/camera_kalibratie.json` (nu: 1; 1,4; 1,62; 2; 2,4 en 3,4 m).
- **De fit:** rechte door (1/d, y) met kleinste kwadraten → `k` en `y_horizon`. Onzin (k ≤ 0, horizon buiten het beeld) wordt geweigerd. Nu: k = 0,154, y_horizon = 0,546.
- **Bereik:** verder dan 6 m ligt de rij te dicht bij de horizon om iets te betekenen.
- **Dode zone:** dichter dan `k / (0,95 − y_horizon)` ziet de camera de vloer niet meer (nu 0,38 m). De stopafstand is daarom nooit kleiner dan die dode zone + 5 cm.

### Wat het robot ermee doet

- **Stopafstand:** standaard 0,4 m (Opties). Ter plaatse draaien beschrijven de hoeken een cirkel van 42 cm rond het midden, 11 cm voorbij de voorkant; de rest is marge voor de camerafout dichtbij en de vertraging. Met de huidige kalibratie wordt het 0,43 m (dode zone 0,38 + 0,05).
- **Vrije weg** = gemeten afstand − stopafstand − snelheid × (leeftijd van het beeld + 0,4 s). Die 0,4 s is de vertraging van camera en wifi: zo ver rijdt het robot al voor het beeld binnen is.
- **Afremmen:** rechtdoor rijden remt naar die vrije weg zoals naar een doel (√(2 · rem · vrije weg)). Is de vrije weg 0: `Obstakel`, motoren uit.
- **Blind = stoppen:** geen vers beeld in 1,5 s → `CameraWeg`. Het robot rijdt nooit blind vooruit.
- **Alleen vooruit:** achteruit en draaien kijkt de camera niet.

**In een parcours** ([rijden/parcours.py](../jaguar/rijden/parcours.py)):

1. tot 4 s wachten: loopt iemand weg (weg vrij tot stopafstand + 1 m), dan gewoon verder;
2. is er naast het obstakel vrije vloer: een **omweg** naar de vrijste kant. 0,5 m achteruit, 45° opzij, schuin tot 0,9 m opzij, 1,2 m rechtdoor naast het obstakel (de diepte ziet de camera niet), schuin terug op de lijn, en weer op koers. Hoogstens 2 omwegen per stap;
3. muur dwars over de weg, of geen omweg mogelijk: de stap eindigt voor het obstakel en het parcours gaat verder met de volgende stap.

**Handmatig rijden:** vooruit is geblokkeerd zolang er iets binnen de stopafstand staat; draaien en achteruit niet.

**Grenzen:** glas, een muur in de kleur van de vloer, een vloer met een sterk patroon of een rand (tapijt), en donker. De noodstop blijft nodig.

**Een eigen model aansluiten** (bv. een neuraal netwerk): `robot.py` gebruikt alleen een object met `latest()` (afstand in m of None, leeftijd van het beeld in s), `stop` (stopafstand) en `error`; `parcours.py` daarnaast `dist` en `free_side(m)`. De oude versie van de camera staat op branch `camera-origineel`.

---

## 9. Veiligheid en batterij

Alle rijlussen gaan door `_motor()` en `_guard()` in `robot.py`. Daar werken STOP, Pauze, Terug naar start, de batterij en deze stops, in elke fase:

| Stop | Wanneer |
|---|---|
| Gekanteld | meer dan 35° t.o.v. de start |
| Vastgelopen | 1,5 s bij (bijna) vol vermogen nauwelijks vooruit |
| Wielen in de lucht / afstand onzeker | langer dan 1 s |
| Naast het pad | meer dan 1 m (pad volgen) |
| Obstakel | binnen de stopafstand vooruit (camera aan) |
| Camera weg | 1,5 s geen beeld (camera aan) |
| Watchdog | geen PING: de controller zet de motoren uit |

Na elke rit, ook na een fout: motoren 0 en de E-Stop weer aan.

**Batterij:** de spanning springt ±0,5 V tussen twee metingen, dus het script kijkt naar de mediaan over 10 s.

- Beginnen kan vanaf 22,2 V.
- Onder 21,6 V (3,6 V per cel) breekt het parcours af en rijdt het robot terug naar de start.
- Onder 21,0 V (3,5 V per cel) stopt het helemaal, ook op de terugweg.

---

## 10. Bediening vanuit de browser

`python -m jaguar.rijden.bediening` (met `--ip 127.0.0.1` tegen een simulator) houdt de live pagina open op http://127.0.0.1:8765.

- **Rijden: de ontwerper** ([live/](../jaguar/live/README.md)). Kies **Binnen** (raster in meter) of **Buiten** (OpenStreetMap/Esri-kaart met coördinaten en de richting bij de start).
  - Tekenen met de muis: de lijn wordt gladgestreken en vereenvoudigd (Ramer-Douglas-Peucker). Een zachte hoek (minder dan 50°) wordt een boog met de grootste straal die past; een scherpe hoek een draai ter plaatse. Kiezen kan met **Auto / Alles rond / Scherp**.
  - Of punten zetten en verslepen, een punt invoegen met de **+** in het midden van een stuk, of **Voorbeelden** (lijn, heen en terug, vierkant, rechthoek, ovaal, cirkel, slalom).
  - De simulatie rijdt het robot op ware grootte over de route, met de benodigde ruimte en de geschatte tijd. Starten pas na een bevestiging ("robot staat goed, ruimte vrij").
- **Pauze** (knop of P): stopt waar het is; **Hervatten** rijdt dezelfde stap verder. Pauzeren mag ook vanaf een ander apparaat, hervatten alleen op de laptop.
- **Terug naar start:** breekt het parcours af, draait naar de start, rijdt er in een rechte lijn heen (achteruit als de start achter het robot ligt en dichter dan 3 m), en draait naar de beginrichting. Kan ook na de rit, met de laatste positie volgens de odometrie. Kijkt niet of de lijn vrij is (alleen de camera vooruit).
- **Handmatig:** pijltjes/WASD, de knoppen op het scherm of een gamepad (linkerstick, B = STOP), met een schuif voor het vermogen. De pagina stuurt 10× per seconde de stand; komt er 0,4 s niets, dan stopt het robot. Rechtdoor houdt de gyro de koers vast. Alleen op de laptop.
- **STOP** (knop of spatie): motoren 0 en E-Stop aan. Werkt vanaf elk apparaat op het wifi.
- De fysieke noodstop blijft de echte noodstop.

---

## 11. Live meekijken en Grafana

- **Live pagina:** elk rijscript start [live/view.py](../jaguar/live/view.py) op poort 8765 (`--no-live` = uit): tegels, grafieken, camera, knoppen. Andere apparaten op het wifi openen `http://<IP-laptop>:8765/` (`--no-network` = alleen lokaal).
- **Opties:** grafieken tonen of verbergen, vanzelf analyseren na de rit, camera, obstakels en stopafstand. Bewaard in `jaguar/live/opties.json`.
- **Grafana:** met `--mqtt` gaan dezelfde waarden naar Mosquitto (poort 1884) → Telegraf → InfluxDB (8086) → Grafana op http://localhost:3000, dashboard "Jaguar live". Topics `jaguar/imu` (50 Hz) en `jaguar/status` (2 Hz). Starten: `cd grafana; docker compose up -d`. Draait Grafana op een andere laptop: `--mqtt-host <IP>`.
- De live waarden zijn een benadering; de analyse van de CSV is de definitieve meting.

---

## 12. Opslaan, analyse en dashboard

**De CSV:** elke rit wordt `ritten/rit_<datum>_<tijd>_<soort>.csv` met vier kolommen: tijd, fase, vermogen en de ruwe regel. Daartussen staan de eigen regels van het script: `PLAN`, `PAD`, `PADPUNTEN` (het gevraagde pad, een punt om de 10 cm), `DOEL`, `DRAAI`, `OBSTAKEL`, `OMWEG`, `MELDING` … Zonder CSV kan achteraf niets meer berekend worden (`--no-csv` = niet opslaan).

**De analyse** (`python -m jaguar.analyse.rit` of de knop **Analyse**):

1. de CSV inlezen: IMU, encoders, motoropdrachten en de eigen regels;
2. de gyrobias en de zwaartekracht uit de laatste 3 s rust;
3. alle regels opnieuw door dezelfde odometrie (§2): afstand, slip, gebeurtenissen;
4. het gereden spoor uit afstand × gyrokoers;
5. helling en hoogte: pitch uit de accelerometer in stilstand, uit de gyro tijdens het rijden; de kanteling door het motorkoppel wordt eraf gerekend;
6. trillingen, snelheid, rechtlijnigheid en "tegengehouden" (vermogen aan, minstens 25 % trager, minstens 3× de normale motorstroom);
7. bij een pad: elk punt van het spoor tegen de gevraagde lijn → gemiddeld en hoogstens ernaast, en hoe ver het van het einde stopte (figuur "Route in één beweging");
8. schrijft `ritten/<rit>_analyse.json` en bouwt `dashboard/dashboard.html` opnieuw met alle ritten.

**Het dashboard** (`dashboard/dashboard.html`, opent zonder server; voor 3D is internet nodig):

- **In het kort:** één alinea en vijf vragen (hoe ver, terug op de start, rechtdoor, slip, iets bijzonders);
- **Bekijk de rit:** afspelen (½× tot 4×) met een tijdlijn, bovenaanzicht en 3D; wielen kleuren geel bij slip en rood als ze doordraaien;
- **Wat gebeurde er onderweg?:** gebeurtenissen in gewone taal, klik om ernaartoe te springen;
- **Hoe weet de robot hoe ver hij reed?:** drie animaties;
- **Technische details:** alle cijfers en grafieken.

Het spoor in het dashboard is waar het robot **denkt** te zijn. Hoe ver dat van de werkelijkheid ligt, meet je met een rolmeter, of in de simulator (§14).

---

## 13. Kalibreren

| Wat | Hoe |
|---|---|
| Wielstraal | `python -m jaguar.rijden.rechtdoor --distance 3`, met de rolmeter meten, dan `python -m jaguar.kalibratie.kalibreer 2.968`. Schrijft `config/kalibratie.json` (nu 0,1336 m; zonder bestand 0,135 m). Op een harde, vlakke vloer. `--vergelijk` toont per methode de fout zonder op te slaan |
| Afstand met de GPS | `python -m jaguar.kalibratie.gps_afstand --distance 20 [--bewaar]`: bewaart alleen als de meting goed genoeg is (met de GPS van het robot vaak niet) |
| Meetwielen | `test_sondewiel adres` (robot op blokken), dan `test_sondewiel rijden --distance 3`: alleen de voorwielen trekken, de achterwielen meten de grond zonder aandrijfslip |
| Gyroschaal | `python -m jaguar.kalibratie.test_sondewiel draai --turns 2`, en de echte hoek ingeven |
| Driver | `python -m jaguar.kalibratie.test_driver` leest de driverinstellingen (alleen lezen) |
| Camera | Opties → Obstakels → Kalibreren… (§8) |

---

## 14. Zonder robot: simulatoren en tests

| Wat | Hoe |
|---|---|
| Nep-robot | `python -m jaguar.testen.nep_robot` speelt `testdata/demo_rit.csv` af (reageert niet op de motoren) |
| Gesimuleerde robot | `python -m jaguar.testen.sim_robot`: vertraging, wrijving, slip bij draaien, watchdog 0,3 s, E-Stop, GPS 5 Hz. Opties `--gps-ruis`, `--gps-dwaal`, `--gps-vertraging`, `--slip`, `--vertraging`, `--terrein`, `--spoor` |
| Websimulator | robotics.dinno.be via de lokale Training API en `api_brug.py` (hieronder) |
| Tests | `test_rijden` (~4 min), `test_gps_route` (~6 min), `test_bediening` (~2 min), `test_navigatie`; `sim_boog` voor de bogen |

Rijden tegen een simulator: zelfde commando's, met `--ip 127.0.0.1`.

### Demo met stenen en plassen

- `python -m jaguar.rijden.bediening --demo` start de gesimuleerde robot in hetzelfde programma (op een vrije poort) met het terrein uit `testdata/terrein_demo.json` (`--terrein geen` = vlakke grond). De pagina toont "DEMO SIMULATIE", en `http://127.0.0.1:8765/kaart` toont live het terrein, de gevraagde route en waar het robot **echt** rijdt. Het robot kijkt bij de start naar het noorden (boven op het raster).
- `python -m jaguar.testen.terrein_rapport --demo` rijdt het demo-parcours, analyseert het en tekent `ritten/terrein_rapport.png` (ongeveer twee minuten). Zonder `--demo` maakt het het rapport van de nieuwste rit.
- Op de tekening: stenen (grijs), plassen (blauw), het echte spoor (rood), het spoor volgens de odometrie (gekleurd naar de trilling) en wat de analyse onderweg zag.
- Wat je eraan ziet: stenen voelt het robot (trilling, schokken) en het rijdt er gewoon overheen. In een plas slippen alle wielen samen: de wielen tellen meer meters dan het robot aflegt, en daarna rijdt het naast zijn route zonder het te weten (§2, grens). Het robot herkent een plas niet vooraf en ontwijkt niets: het volgt de getekende lijn.

### De websimulator via de lokale API

```text
rijscript ──TCP 127.0.0.1:10001──> testen/api_brug.py ──HTTP localhost:8780──> robotSimulatorAPI (server.mjs) ──> simulator in Chrome
```

- **De API:** [github.com/mdequanter/robotSimulatorAPI](https://github.com/mdequanter/robotSimulatorAPI), ongewijzigd (commit `585e8f6`). `node server.mjs --show` draait de simulator van de site in Chrome, met de sleutel van je account in `ROBOTSIM_KEY`.
- **De brug** ([testen/api_brug.py](../jaguar/testen/api_brug.py)) doet zich voor als het echte robot:
  - vraagt 50 stappen van 0,02 s per seconde (`POST /api/envs/{id}/step`), in echte tijd;
  - geeft onze regels (`PING`, `MMW !M ..`) mee als actie `{"tcp": [...]}`;
  - bouwt uit elke waarneming de regels van het echte robot: IMU (50 Hz), `MM0/MM1 C=` en `V=`, `$GPRMC` uit het veld `gps` (5 Hz), `FF=` (1 Hz);
  - keert het teken van de gyro-Y-as om (in de simulator andersom dan op het robot; zonder dat telde de odometrie op een heuvel de helling dubbel en verwierp ze de wielen);
  - zet het robot neer met `--plaats x,z,koers` (actie `place`; in de simulator 0° = oost, 90° = noord);
  - bewaart met `--spoor` 5× per seconde de **echte** positie (veld `pose`) en met `--beeld` een JPEG van het einde.
- **Starten:** `.\start_websim.ps1 -Plaats "-3.4,0,90"` doet beide. Daarna `$env:JAGUAR_WIELSTRAAL = "0.135"` (de wielstraal van de simulator) en rijden met `--ip 127.0.0.1`.
- **De GPS in de simulator** heeft een fout van ongeveer 1 m die langzaam dwaalt, en een resolutie van 18 cm.

**Drie posities in de simulator:**

| Bron | Ziet onze code het? | Waarvoor |
|---|---|---|
| GPS van de simulator (`$GPRMC`) | ja | noorden en schaal |
| Encoders en IMU van de simulator | ja | sturen |
| Echte positie (`pose`) | nee | achteraf meten (`ritten/websim_spoor.csv`) |

**De ε-baan** (bord 12 in het park): 87 GPS-punten om de 0,5 m langs de middellijn van het aarden pad, 42 m, in `testdata/websim_epsilon_baan.txt`. Gereden met `gps_route --vloeiend --richting 0`: gemiddeld 4,9 cm en hoogstens 12,3 cm naast de middellijn, gestopt op 8,5 cm van het doel (echte positie). `docs/websimulator/teken_baan.py` tekent het spoor tegen de baan.

---

## 15. Bestanden in config/

| Bestand | Inhoud |
|---|---|
| `kalibratie.json` | gemeten wielstraal (weg = 0,135 m) |
| `leerwaarden.json` | bijgeleerde waarden: vermogen→snelheid, uitloop, kruipstoot, draaistoot (niet in git) |
| `camera_kalibratie.json` | de kalibratiepunten van de camera en het model (k, y_horizon, dode zone) |

`JAGUAR_WIELSTRAAL` in de omgeving gaat voor `kalibratie.json` (voor de websimulator: 0.135). `JAGUAR_DRIVER=0` zet de positiemodus van de driver uit.
