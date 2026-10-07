# Mathematical models — documentation

| File | Content |
|---|---|
| `Mathematical_Models.pdf` | 42-page learning guide. It starts with the big picture and a maths refresher, and ends with recap tables and a glossary. Every equation of both flows has a boxed statement, a table of symbols with dimensions and units, a derivation, a hand-checkable toy example and a plain-language "in our drone project" explanation |
| `figures/flow_model1.pdf` / `.png` | Model 1 equation flow (nonlinear dynamics → EKF → SO(3) control), with input/output on every block and a notation-and-dimensions panel |
| `figures/flow_model2.pdf` / `.png` | Model 2 equation flow (flight data → DMDc → LQR / MPC → drone), same format |

Rebuild (MiKTeX or TeX Live): run `pdflatex flow_model1.tex` and `pdflatex flow_model2.tex` twice each inside `figures/`, then `pdflatex Mathematical_Models.tex` twice. The result plots are read directly from `../../toy_model2/*/figures/`.
