"""Een route van GPS-coördinaten omzetten naar parcoursstappen (draaien en rechtdoor).

De hoekpunten komen uit de opgegeven coördinaten, niet uit de GPS van het robot: de route zelf is dus exact
(de afstanden en hoeken tussen de punten) en niet afhankelijk van de GPS-ruis. Alleen de richting van het
noorden is nodig om te weten hoe het robot bij de start moet staan; die komt uit de COG of wordt opgegeven.

Tussen twee hoekpunten rijdt het robot één rechte lijn zoals parcours.py dat doet (naar het geplande
hoekpunt, met koerscorrectie); extra tussenpunten om de 2 m helpen niet en kosten alleen draaiingen.
"""
import math

from jaguar.navigatie.gps import Lokaal, afstand_richting, hoek180


def route_naar_stappen(punten, b0, lokaal=None, begin_koers=0.0):
    """punten: lijst (lat, lon); het eerste is coördinaat 1 = het nulpunt, waar het robot start.
    b0: kompaskoers (graden) waarin het robot bij de start kijkt (0 = noord, 90 = oost).
    Geeft (stappen, lengtes): stappen zoals parcours.rijd ze verwacht [("links"|"rechts"|"vooruit", waarde)],
    en voor elke rechte stap de afstand (m)."""
    if len(punten) < 2:
        raise ValueError("een route heeft minstens twee punten")
    lokaal = lokaal or Lokaal(*punten[0])
    xy = [lokaal.naar_xy(lat, lon) for lat, lon in punten]
    koers = begin_koers                    # robotkoers (graden, + = links); kompaskoers = b0 - koers
    stappen, lengtes = [], []
    for van, naar in zip(xy, xy[1:]):
        afstand, peiling = afstand_richting(van, naar)
        if afstand < 0.01:
            continue                       # hetzelfde punt twee keer
        nodig = hoek180((b0 - peiling) - koers)    # links positief
        if abs(nodig) >= 0.5:
            stappen.append(("links" if nodig > 0 else "rechts", abs(nodig)))
            koers += nodig
        stappen.append(("vooruit", afstand))
        lengtes.append(afstand)
    return stappen, lengtes


def beschrijf(stappen):
    return " → ".join(f"{s} {v:.2f} {'m' if s in ('vooruit', 'achteruit') else '°'}" for s, v in stappen)
