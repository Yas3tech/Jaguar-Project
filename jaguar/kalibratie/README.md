# kalibratie/ — meten hoe de robot echt rijdt

- `kalibreer.py`: wielstraal kalibreren met een rolmeter, of de afstandsmethodes vergelijken. Schrijft `config/kalibratie.json`.
- `test_sondewiel.py`: proeven met vrijlopende meetwielen (robot op blokken) en de gyroschaal (`draai`).
- `test_driver.py`: leest de instellingen van de motordrivers uit (alleen lezen, niets beweegt).

```
python -m jaguar.kalibratie.kalibreer 2.97 --vergelijk
python -m jaguar.kalibratie.test_sondewiel draai --turns 2
```
Uitleg staat in de hoofd-README, onder "Afstand kalibreren".
