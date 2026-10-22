# analyse/ — een rit achteraf bekijken

- `rit.py`: leest een CSV uit `ritten/`, rekent afstand, koers, slip en gebeurtenissen uit, schrijft `ritten/<rit>_analyse.json` en werkt `dashboard/dashboard.html` bij. Bevat ook de constanten die de rest van de code gebruikt (wielstraal, encoderpuls, gyroschaal).

```
python -m jaguar.analyse.rit                          # nieuwste rit
python -m jaguar.analyse.rit ritten/rit_....csv       # een bepaalde rit
```
