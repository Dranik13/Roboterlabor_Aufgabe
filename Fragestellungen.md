# Frage 1: Kann der Lösungspfad mit einem Industrieroboter abgefahren werden? Welche Bewegungsbefehle könnten sie einsetzen?

Da die gefundenen Pfade grundsätzlich kollisionsfrei sind, können diese von einem Industrieroboter abgefahren werden. Des Weiteren führt die Glättung noch zu einer Optimierung des Bewegungsablaufs und somit zu einer Verhinderung von unnötigen Bewegungen oder Verdrehungen.

Es können die nachfolgenden Bewegungsbefehle eingesetzt werden:
- PTP: Einfaches Punkt-zu-Punkt-Anfahren der Knoten mit oder ohne Überschleifen
- SPLINES: Insbesondere für bspw. die Kurven
- SPTP: Für lineares Anfahren der Punkte. Die Bahn ist vorhersehbarer als bei reinem PTP.

Wenn jedoch ein reines PTP, also die lineare Einschränkung fehlt, genutzt wird, dann kann es unter Umständen sein, dass der Pfad nicht mehr kollisionsfrei ist. Dies liegt daran, dass bei einem reinen PTP die Bewegung von einem Punkt zum nächsten nicht vorhersehbar ist. Da die Planer und Smoother jedoch immer lineare Punkt-zu-Punkt-Bewegungen vorausgesetzt haben, muss in diesem Fall ein SPTP, also ein lineares PTP, genutzt werden. Hierbei bewegt sich der Effektor auf einer linearen räumlichen Bahn zum Zielknoten. Die Bewegung wird somit vorhersehbar, da sich auch der restliche Körper im gleichen Maße bewegt. Bei einem reinen PTP kann dies unter Umständen nur dann gewährleistet sein, wenn die Wegpunkte sehr dicht aneinanderliegen und so gut wie keine andere Lösung, als die lineare zulassen.

# Frage 2: Was bremst die Abfahrgeschwindigkeit noch aus?

Im Falle des ungeglätteten Pfades wird die Abfahrgeschwindigkeit aufgrund von unnötigen Rotationen oder Umwegen ausgebremst, was nach der Glättung nicht mehr der Fall ist. Allerdings gibt es Einschränkungen durch den verwendeten Bewegungsbefehl.

Im Falle des PTP:
- Reines PTP und SPTP fahren immer von Punkt zu Punkt. Das bedeutet, dass es bei jedem Punkt einen vollen Stopp vollzieht, bevor der nachfolgende angefahren wird. Dies ist nicht nur energetisch und verschleißtechnisch, sondern auch in Bezug auf die Geschwindigkeit der Aufgabenerfüllung unvorteilhaft.
- Je dichter hierbei die Wegpunkte liegen, desto langsamer wird die Bewegung bis zum völligen Stillstand, da die Distanz zu klein für eine großartige Bewegung (Beschleunigungen und Abbremsungen) werden.

# Frage 3: Wie könnte man weiterhin die Abfahrgeschwindigkeit erhöhen?

Beim reinen PTP kann man dem unnötigen Anhalten an jedem Knoten mit dem einsetzen eines sogenannten "Überschleifens" entgegenwirken. Hierbei wird ein Entfernungsradius/Distanzwert zum aktuell anzufahrenden Knotenpunkt vorgegeben, ab dessen Unterschreitung der darauffolgende Punkt angefahren werden soll. Hierdurch wird das Abbremsen reduziert oder ganz verhindert und eine gleichmäßige Bewegung ermöglicht. Jedoch ist darauf zu achten das Überschleifen nicht zu stark zu gestalten, da sonst die Kollisionsfreiheit der Bewegung gefährdet werden kann. 

Im Falle von sehr dicht liegenden Wegpunkten kann das Überschleifen jedoch problematisch werden, da der Abstand der Punkte nicht ausreichend sein kann. In diesem Szenario können die Knoten zur Modellierung einer SPLINE-Bewegung genutzt werden, die das elegante Abfahren von Kurven ermöglicht und die Kollisionsfreiheit, insbesondere bei sehr vielen Stützpunkten gewährleistet.

