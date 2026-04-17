# ReactionESP Funktionsbeschreibung

ReactionESP ist ein modulares System aus mehreren vernetzten Reaktionslichtern. Jedes Reaktionslicht basiert auf einem ESP8266/D1 mini pro und kommuniziert mit den anderen Geräten über ESP-NOW. Das System ist zunächst für 2 Geräte ausgelegt und soll später auf bis zu 8 Reaktionslichter erweitert werden.

Ein Reaktionslicht kann abhängig von seiner Ausstattung auf unterschiedliche Sensoren reagieren. Vorgesehen sind insbesondere Tap-Erkennung über einen Beschleunigungssensor, Berührungserkennung, Druck- oder Bewegungserkennung sowie RFID/NFC zur Auswahl von Spielmodi. Die sichtbare Rückmeldung erfolgt über RGB-LEDs, zusätzlich kann ein Display Statusinformationen wie MAC-Adresse, Gruppe, Rolle oder Spielzustand anzeigen.

## Rollen und Gruppen

Die Rolle eines Geräts wird lokal über einen Schalter an Pin 16 bestimmt. Ist der Schalter aktiv, arbeitet das Gerät als Master. Andernfalls arbeitet es als normales Reaktionslicht.

Der Master koordiniert das Spiel, verarbeitet Ereignisse der Reaktionslichter und sendet neue Lichtzustände an die Teilnehmer. Ein normales Reaktionslicht meldet Sensorereignisse wie einen Tap an den Master und setzt die vom Master empfangenen Lichtzustände um.

Über einen Schalter an Pin 14 wird jedes Gerät einer von zwei Gruppen zugeordnet:

- Gruppe A
- Gruppe B

Diese Gruppen ermöglichen zwei Betriebsarten. Entweder laufen zwei getrennte Spiele parallel, zum Beispiel mit je einem RFID-fähigen Master pro Gruppe, oder die Gruppen werden als zwei Teams innerhalb eines gemeinsamen Spiels genutzt. Nachrichten tragen eine Gruppenkennung, sodass ein Gerät Nachrichten der anderen Gruppe ignorieren kann. Für spätere globale Befehle ist außerdem eine gruppenübergreifende Zielgruppe vorgesehen.

## RFID/NFC-Spielauswahl

Mindestens ein Gerät kann mit einem RFID/NFC-Leser ausgestattet werden. Dieses Gerät soll Spielmodi über RFID/NFC-Tags auswählen. In einem Gruppensystem kann jede Gruppe einen eigenen RFID-fähigen Master besitzen. Dadurch kann Gruppe A ein anderes Spiel oder eine andere Runde starten als Gruppe B.

Ein RFID/NFC-Tag soll nicht direkt ein Licht auslösen, sondern einen Spielmodus oder eine Spielkonfiguration wählen. Der Master übersetzt den gelesenen Tag in einen Spielzustand und verteilt anschließend Start-, Stop- oder Konfigurationsnachrichten an die zugehörige Gruppe.

## Umgesetzte Grundlagen

Aktuell sind die wichtigsten technischen Bausteine im Aufbau:

- ESP-NOW-Kommunikation zwischen Master und Reaktionslichtern
- lokale Master-Auswahl über Pin 16
- lokale Gruppenauswahl über Pin 14
- Gruppenkennung in den Nachrichten
- Filterung empfangener Nachrichten nach Gruppe
- Tap-Erkennung über den ADXL345
- Ausgabe über NeoPixel-LEDs
- vorbereitete RFID/NFC-Anbindung über PN532
- vorbereitete Struktur für bis zu 8 Reaktionslichter
- aktive State-Machine mit Init-State
- Anmeldung der Reaktionslichter beim Master
- Verteilung der aktiven `stateId` durch den Master
- erster Spiel-State "Klassiker: 1 aus n"

Die erste Spiellogik ist in klar getrennte Spielmodi ueberfuehrt. Der Klassiker "1 aus n" nutzt getrennte Master- und Light-States, waehrend Sensor- und Netzwerkereignisse ueber die State-Machine verarbeitet werden.

## State-Architektur

Auf allen Geraeten soll das gleiche Programm laufen. Die konkrete Aufgabe eines Geraets ergibt sich erst zur Laufzeit aus seiner Rolle, seiner Gruppe, seiner Ausstattung und der vom Master verteilten State-ID.

Alle Geraete starten im Init-State. In diesem Zustand initialisieren sie Hardware, ESP-NOW, Sensoren, Anzeige und Gruppen-/Rolleninformationen. Normale Reaktionslichter melden sich anschliessend beim Master an und warten auf eine `stateId`. Erst wenn der Master eine gueltige `stateId` verteilt, wechseln die Geraete in den passenden Spiel-State.

Die einzelnen Spiele sollen jeweils als eigene States abgebildet werden. Sensorereignisse werden nicht direkt in der Hauptschleife fest verdrahtet, sondern als Events an den aktuell aktiven State uebergeben. Beispiele fuer solche Events sind `Tap`, `Touch`, Druckaenderung, RFID/NFC gelesen, Timeout oder ESP-NOW-Nachricht empfangen.

Jedes Spiel besteht aus zwei zusammengehoerigen State-Implementierungen:

- einem State fuer normale Reaktionslichter
- einem State fuer den Master

Beide States teilen sich einen gemeinsamen Header. In diesem Header werden gemeinsame Konstanten, State-ID, Nachrichtenstrukturen, Event-IDs und Hilfsfunktionen des Spiels definiert. Dadurch bleibt klar, welche Nachrichten und Regeln zu einem Spiel gehoeren, waehrend Master-Logik und Licht-Logik getrennt implementiert werden koennen.

Der Master-State verwaltet den Spielzustand, wertet Sensor- und Netzwerkereignisse aus und sendet Sollzustaende an die Reaktionslichter. Der Light-State setzt den empfangenen Zustand lokal um, liest seine Sensoren und meldet relevante Events an den Master.

Der Master muss ausserdem zyklisch in einem eigenen Task oder einer klar getrennten Polling-Funktion den RFID/NFC-Reader abfragen. Systembedingt kann das Programm beim RFID/NFC-Lesen an dieser Stelle blockieren, bis eine Karte gelesen wurde. Diese Blockade darf deshalb nicht unkontrolliert in die allgemeine Spiellogik wandern. Sie soll in der Master-RFID-Aufgabe gekapselt werden, damit die Spiel-States und die Kommunikation so nachvollziehbar wie moeglich bleiben.

## Spiele für die Reaktionslichter

### Klassiker: 1 aus n

Status: erste umgesetzte Spiellogik.

Bei diesem Spiel leuchtet immer genau eines von n Reaktionslichtern. Der Spieler muss das leuchtende Reaktionslicht möglichst schnell mit einem Tap berühren. Sobald der Tap erkannt wurde, wird dieses Licht ausgeschaltet und ein zufälliges anderes Reaktionslicht eingeschaltet. Danach wiederholt sich der Ablauf.

Der Master verwaltet dabei die aktive Licht-ID, wählt nach jedem erfolgreichen Tap ein neues Licht aus und sendet den neuen Zielzustand an alle Geräte der Gruppe. Das gerade aktive Licht leuchtet in einer klar definierten Farbe, alle anderen Lichter bleiben aus oder zeigen optional einen Bereitschaftszustand.

Wichtige Regeln:

- Zu jedem Zeitpunkt ist pro Gruppe nur ein Reaktionslicht aktiv.
- Nur ein Tap auf dem aktiven Licht zählt als Treffer.
- Ein Tap auf einem nicht aktiven Licht wird ignoriert oder optional als Fehler gewertet.
- Das nächste Licht wird zufällig gewählt, darf aber nicht identisch mit dem gerade getroffenen Licht sein.
- Das Spiel kann für Gruppe A und Gruppe B unabhängig laufen.

Mögliche Wertungen:

- Anzahl Treffer innerhalb einer festen Zeit
- Zeit bis zu einer festen Anzahl Treffer
- durchschnittliche Reaktionszeit
- schnellste und langsamste Reaktion
- Fehleranzahl bei falschen Taps

### Team-Duell A gegen B

Status: geplant.

In diesem Modus spielen Gruppe A und Gruppe B gegeneinander. Jede Gruppe hat ihre eigenen Reaktionslichter. Pro Gruppe leuchtet jeweils ein Licht. Sobald ein Spieler das aktive Licht seiner Gruppe trifft, bekommt das Team einen Punkt und das nächste Licht der gleichen Gruppe wird gewählt.

Das Spiel kann entweder mit einem gemeinsamen Master für beide Gruppen oder mit je einem Master pro Gruppe umgesetzt werden. Bei zwei Mastern sollten beide Gruppen unabhängig laufen, während ein globaler Start oder Stop später über eine gruppenübergreifende Nachricht möglich ist.

Mögliche Wertungen:

- erste Gruppe mit einer Zielpunktzahl gewinnt
- höchste Punktzahl nach Ablauf der Zeit gewinnt
- Bonus für Trefferfolgen ohne Fehler

### Reaktionszeit-Test

Status: geplant.

Ein zufälliges Licht geht nach einer zufälligen Wartezeit an. Der Spieler muss es so schnell wie möglich antippen. Der Master misst die Zeit zwischen Einschalten des Lichts und Eingang des Tap-Ereignisses.

Dieser Modus eignet sich für Einzelspieler, Training und Vergleichsmessungen. Bei Gruppenbetrieb kann jede Gruppe separat eigene Messwerte erfassen.

Mögliche Wertungen:

- beste Reaktionszeit
- Durchschnitt aus mehreren Versuchen
- Auswertung von Frühstarts
- Anzeige der letzten Reaktionszeit am Master-Display

### Runden-Timer

Status: geplant.

Der Runden-Timer ist ein Workout-Timer mit Ampelanzeige. Er steuert Trainingsintervalle, bei denen alle beteiligten Reaktionslichter den aktuellen Phasenstatus anzeigen. Dadurch kann das System auch ohne direkte Tap-Aufgabe als gut sichtbarer Trainings-Timer genutzt werden.

Die Ampelfarben haben feste Bedeutungen:

- Gelb blinkend: Vorbereitung vor dem Start
- Gruen: Arbeitsphase
- Rot: Pause

Beim Start blinkt Gelb fuer 5 Sekunden. Danach beginnt die erste Arbeitsphase. Als Default dauert eine Arbeitsphase 40 Sekunden, gefolgt von 20 Sekunden Pause. Nach der Pause startet automatisch die naechste Arbeitsphase, bis die konfigurierte Anzahl Runden beendet ist oder der Timer gestoppt wird.

Wichtige Regeln:

- Alle Lichter einer Gruppe zeigen synchron die gleiche Timerphase.
- Gelb blinkend wird zur Vorbereitung beim Start genutzt.
- Gruen bedeutet: arbeiten, trainieren oder Aufgabe ausfuehren.
- Rot bedeutet: Pause oder Wechselzeit.
- Der Master verwaltet die Zeit und sendet Phasenwechsel an die Lichter.

Default-Konfiguration:

- Vorbereitung: 5 Sekunden, gelb blinkend
- Arbeit: 40 Sekunden, gruen
- Pause: 20 Sekunden, rot
- Rundenanzahl: noch festzulegen

Moegliche Erweiterungen:

- RFID/NFC-Tag zur Auswahl verschiedener Workout-Profile
- Anzeige der verbleibenden Zeit am Master-Display
- akustisches Signal bei Phasenwechsel
- getrennte Timerprofile fuer Gruppe A und Gruppe B
- Countdown-Blinken in den letzten 3 Sekunden einer Phase

### Memory-Sequenz

Status: geplant.

Die Reaktionslichter zeigen eine Sequenz aus Farben oder Positionen. Der Spieler muss die Reihenfolge anschließend durch Taps wiederholen. Nach jeder erfolgreichen Runde wird die Sequenz länger.

Der Master speichert die Sequenz und prüft die eingehenden Tap-Ereignisse. Falsche Eingaben beenden die Runde oder ziehen Punkte ab.

Mögliche Varianten:

- reine Positionssequenz
- Farbsequenz
- Team-Modus mit getrennten Sequenzen für Gruppe A und Gruppe B
- kooperativer Modus, bei dem mehrere Spieler gemeinsam eine Sequenz lösen

### Trefferjagd

Status: geplant.

Mehrere Reaktionslichter können gleichzeitig oder kurz nacheinander aufleuchten. Die Spieler müssen möglichst viele aktive Lichter innerhalb einer Zeitspanne treffen.

Dieser Modus ist hektischer als der Klassiker und eignet sich für Training von Orientierung, Bewegung und Reaktionsvermögen. Der Master entscheidet, wie viele Lichter gleichzeitig aktiv sein dürfen und wann neue Ziele erscheinen.

Mögliche Wertungen:

- Treffer pro Minute
- verpasste Ziele
- Fehler bei inaktiven Lichtern
- Teamwertung nach Gruppe

### Capture the Flag

Status: geplant.

Bei diesem Spiel werden die Reaktionslichter ohne eigenen RFID/NFC-Leser als bewegliche Flags verwendet. Im Boden dieser Lichter befindet sich jeweils ein RFID/NFC-Tag. Die RFID-faehigen Geraete dienen als Bases der Teams, zum Beispiel eine Base fuer Gruppe A und eine Base fuer Gruppe B.

Ein Spieler kann eine Flag aufnehmen und auf die eigene Base legen. Die Base liest den RFID/NFC-Tag im Boden der Flag und ordnet diese Flag dem eigenen Team zu. Dadurch wird die Flag mit der Teamfarbe der Base "ueberschrieben". Das bedeutet: Die LED der Flag leuchtet danach in der Farbe des Teams, das sie zuletzt auf seiner Base registriert hat.

Der Master oder die jeweilige Base verwaltet dazu fuer jede Flag einen Besitzstatus. Sobald eine Base einen Flag-Tag erkennt, sendet sie eine Nachricht an die zugehoerige Gruppe oder an alle Teilnehmer, damit der neue Besitzer und die neue Farbe sichtbar werden.

Wichtige Regeln:

- Jedes Flag-Licht besitzt eine eindeutige RFID/NFC-Kennung im Boden.
- RFID-faehige Geraete arbeiten als Bases und koennen Flags einnehmen.
- Eine Flag gehoert immer dem Team, dessen Base sie zuletzt gelesen hat.
- Die Teamfarbe der Base bestimmt die Farbe der Flag.
- Flags koennen mehrfach den Besitzer wechseln.
- Das Spiel kann mit zwei Teams oder mehreren Bases erweitert werden.

Mögliche Wertungen:

- Anzahl kontrollierter Flags nach Ablauf der Zeit
- erstes Team mit einer Zielanzahl kontrollierter Flags gewinnt
- Punkte pro gehaltenem Flag und Zeitintervall
- Bonus fuer das Zurueckerobern gegnerischer Flags
- Strafpunkte, wenn eine Flag zu lange keiner Base zugeordnet ist


## Geplante technische Erweiterungen

Für die nächsten Ausbaustufen sind folgende Punkte sinnvoll:

- eindeutige Geräteverwaltung über vollständige MAC-Adressen
- automatische oder konfigurierbare Light-ID-Vergabe
- klarer Nachrichtentyp für Event, Lichtzustand, Spielmodus und Heartbeat
- Spielmodus-Verwaltung ueber eine State Machine
- Anmeldung der Reaktionslichter beim Master im Init-State
- Verteilung der aktiven `stateId` durch den Master
- je Spiel ein gemeinsamer Header fuer Master-State und Light-State
- Event-Uebergabe von Sensoren und ESP-NOW an den aktiven State
- gekapselte RFID/NFC-Abfrage in einem zyklischen Master-Task
- RFID/NFC-Tag-Tabelle zur Spielauswahl
- getrennte Spielzustände pro Gruppe
- Timeout- und Fehlerbehandlung bei verlorenen ESP-NOW-Nachrichten
- Anzeige von Rolle, Gruppe, Spielmodus und Punktestand auf dem Master-Display

## Zielbild

Das Ziel ist ein flexibles Reaktionslicht-System, bei dem jedes Gerät eine klare Rolle und Gruppenzugehörigkeit besitzt. Ein oder mehrere Master koordinieren Spiele, RFID/NFC-Tags wählen Modi aus, und die Reaktionslichter melden Sensorereignisse zuverlässig per ESP-NOW zurück.

Die erste stabile Spielfunktion soll der Klassiker "1 aus n" sein. Darauf aufbauend können Teamspiele, Reaktionszeitmessungen, Sequenzspiele und Sensor-Mix-Varianten ergänzt werden.
