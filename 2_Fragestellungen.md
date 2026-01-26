# Frage 1: Kann der Lösungspfad mit einem Industrieroboter abgefahren werden? Welche Bewegungsbefehle könnten sie einsetzen?

Da die gefundenen Pfade kollisionsfrei sind, können diese grundsätzlich von einem Industrieroboter abgefahren werden. Darüber hinaus führt die Glättung zu einer Optimierung des Bewegungsablaufs und verhindert so unnötige Bewegungen oder Verdrehungen.

Es können die nachfolgenden Bewegungsbefehle eingesetzt werden:
- PTP: Einfaches Punkt-zu-Punkt-Anfahren der Knoten mit oder ohne Überschleifen
- SPTP: Für lineares Anfahren der Punkte. Die Bahn ist vorhersehbarer als bei reinem PTP.

Wenn reines PTP genutzt wird, kann der Pfad unter Umständen nicht mehr kollisionsfrei sein. Dies liegt daran, dass bei einem reinen PTP die Bewegung von einem Punkt zum nächsten nicht vorhersehbar ist. Da die Planer und Smoother jedoch auf linearen Punkt-zu-Punkt-Bewegungen basieren, sollte in diesem Fall ein SPTP (ein lineares PTP) verwendet werden. Hierbei bewegt sich der Effektor auf einer linearen räumlichen Bahn zum Zielknoten. Die Bewegung wird somit vorhersehbar, da sich auch der restliche Körper synchron mitbewegt. Bei einem reinen PTP kann dies nur gewährleistet sein, wenn die Wegpunkte sehr dicht beieinander liegen und kaum eine andere Lösung als die lineare zulassen.

# Frage 2: Was bremst die Abfahrgeschwindigkeit noch aus?

Im Falle des ungeglätteten Pfades wird die Abfahrgeschwindigkeit aufgrund von unnötigen Rotationen oder Umwegen ausgebremst, was durch die Glättung reduziert wird. 

PTP und SPTP Bewegungsbefehle haben eine Eintschränkung:
- Reines PTP und SPTP fahren immer von Punkt zu Punkt. Das bedeutet, dass bei jedem Punkt ein vollständiger Stopp vollzogen wird, bevor der nachfolgende angefahren wird. Dies ist nicht nur energetisch und verschleißtechnisch, sondern auch in Bezug auf die Geschwindigkeit der Aufgabenerfüllung unvorteilhaft.
- Je dichter die Wegpunkte aneinander liegen, desto langsamer wird die Bewegung, da kein Raum für Beschleunigungen vorhanden ist.

# Frage 3: Wie könnte man weiterhin die Abfahrgeschwindigkeit erhöhen?

Bei reinem PTP kann man dem unnötigen Anhalten an jedem Knoten durch den Einsatz eines sogenannten "Überschleifens" entgegenwirken. Dabei wird ein Entfernungsradius zum aktuellen Zielpunkt definiert. Sobald dieser unterschritten wird, wird der nächste Punkt angefahren. Dies reduziert oder verhindert es das Abbremsen und ermöglicht eine gleichmäßigere Bewegung. Allerdings ist zu beachten, dass das Überschleifen nicht zu aggressiv gestaltet werden sollte, da sonst die Kollisionsfreiheit gefährdet wird.

Bei sehr dicht beieinander liegenden Wegpunkten kann das Überschleifen jedoch problematisch werden, da der Abstand der Punkte zu gering sein kann. In diesem Fall können die Knoten zur Modellierung einer SPLINE-Bewegung genutzt werden, die das elegante Abfahren von Kurven ermöglicht und die Kollisionsfreiheit, besonders bei vielen Stützpunkten, gewährleistet.