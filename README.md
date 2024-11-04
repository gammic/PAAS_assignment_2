Aggiunta YAW opzionale:
L'angolo di imbardata è possibile aggiungerlo per migliorare rappresentazione e previsioni della traiettoria della persona. Per implementarlo bisogna aggiungere al vettore di stato originale anche l'angolo di imbardata. A questo punto bisogna aggiornare le matrici presenti nel processo del kalman filter facendo in modo di rispecchiare le dimensioni del vettore di stato.
Questa modifica inoltre porta con sè modifiche nel file tracklet e tracker per creare o aggiornare tracklet, che ora possiedono anche il parametro di imbardata.
Quindi è necessario aggiungere un metodo nel CloudManager che permetta di calcolare l'angolo di imbardata, e viene fatto tramite calcolo della componente principale con PCA, che ci permette di calcolare l'angolo rispetto agli assi.
