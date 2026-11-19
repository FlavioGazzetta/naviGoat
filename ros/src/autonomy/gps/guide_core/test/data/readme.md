# Canned Nominatim responses

Real responses from https://nominatim.openstreetmap.org/search, saved once on
15 Nov 2026 with the project User-Agent
(`NaviGoat-guide/0.7 (+https://github.com/FlavioGazzetta/naviGoat)`), so tests
never use the network. Do not re-download them in a loop: the usage policy
(https://operations.osmfoundation.org/policies/nominatim/) forbids repeated
identical queries.

Parameters: `format=jsonv2 limit=5 addressdetails=1 accept-language=en
viewbox=-0.185,51.505,-0.165,51.490 bounded=1`.

| File | Query | Results |
|---|---|---|
| nominatim_science_museum.json | science museum | 1 |
| nominatim_imperial_college.json | imperial college | 1 |
| nominatim_museum.json | museum | 5 |
| nominatim_empty.json | zzqx no such place | 0 |

Data (c) OpenStreetMap contributors, available under the Open Database
License (ODbL): https://www.openstreetmap.org/copyright