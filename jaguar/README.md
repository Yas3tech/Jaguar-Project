# jaguar/ — alle Python-code

Starten doe je altijd vanuit de projectmap met `python -m jaguar.<map>.<bestand>`.

| Map | Waarvoor | Starten |
|---|---|---|
| [kern/](kern/README.md) | De basis: verbinding met de robot en de berekening van afstand en koers. Start je niet zelf. | (wordt geïmporteerd) |
| [rijden/](rijden/README.md) | De scripts waarmee de robot rijdt | `python -m jaguar.rijden.rechtdoor` |
| [analyse/](analyse/README.md) | Een gereden rit achteraf analyseren en het dashboard maken | `python -m jaguar.analyse.rit` |
| [kalibratie/](kalibratie/README.md) | Wielstraal en gyroschaal meten | `python -m jaguar.kalibratie.kalibreer 2.97` |
| [live/](live/README.md) | De webpagina waarmee je tijdens het rijden meekijkt | start vanzelf bij elk rijscript |
| [testen/](testen/README.md) | Nep-robot om zonder robot te testen | `python -m jaguar.testen.nep_robot` |

Afhankelijkheden: `rijden`, `kalibratie` en `live` gebruiken `kern`; `kern` gebruikt de constanten uit `analyse/rit.py`.
