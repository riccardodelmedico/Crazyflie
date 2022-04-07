# Crazyflie
Questa versione contiene i file necessari all'analisi delle accelerazioni
del drone. In particolare, sono presenti due file aggiuntivi:
1) `SCRIPTS/Accelerometer.py`
2) `MATLAB scripts/accelerometers.m`

Nello script `accelerometers.m` è già presente il file principale
utilizzato per l'analisi. E' quindi sufficiente far girare lo script.
Se fossero necessari nuovi dati, possono essere eseguiti esperimenti utilizzando
lo script `Accelerometer.py`.

Nello script `smoothing.m` è presente una flag aggiuntivo (`prova`)
il cui valore determina se utilizzare velocità e accelerazioni ottenute tramite
puro rapporto incrementale, o utilizzando la funzione MATLAB `smooth()`.