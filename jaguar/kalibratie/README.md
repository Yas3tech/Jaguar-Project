# kalibratie/ — meten hoe de robot echt rijdt

- `kalibreer.py`: wielstraal kalibreren met een rolmeter, of de afstandsmethodes vergelijken. Schrijft `config/kalibratie.json`.
- `gps_afstand.py`: controleert of een gevraagde meter een echte meter is, met de GPS als meetlat. Rijdt de afstand, legt het gereden spoor op het GPS-spoor en leest de schaal af. Met `--bewaar` past het de wielstraal alleen aan als de meting goed genoeg is (anders zegt het dat de meting te ruw is en bewaart het niets). Met de GPS van het robot (meters fout) is een paar meter te weinig: gebruik dan de rolmeter (`kalibreer.py`).
- `test_sondewiel.py`: proeven met vrijlopende meetwielen (robot op blokken) en de gyroschaal (`draai`).
- `test_driver.py`: leest de instellingen van de motordrivers uit (alleen lezen, niets beweegt).

```
python -m jaguar.kalibratie.kalibreer 2.97 --vergelijk
python -m jaguar.kalibratie.gps_afstand --distance 20
python -m jaguar.kalibratie.gps_afstand --distance 20 --bewaar
python -m jaguar.kalibratie.test_sondewiel draai --turns 2
```

Het verschil tussen de rolmeter en de GPS: de GPS-fout op de schaal is ongeveer GPS-fout ÷ afstand. Met een goede GPS (de websimulator of RTK, centimeters) volstaat 10 m. Kalibreer op de ondergrond waarop je gaat rijden.

Uitleg over de rolmeter staat in de hoofd-README, onder "Afstand kalibreren".
