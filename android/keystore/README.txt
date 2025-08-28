Dieser Ordner enthält den Release-Keystore (lokal, nicht einchecken).

Erzeugen (Windows, Voraussetzung: JDK mit `keytool` im PATH):

  keytool -genkeypair -v -keystore dualpid-release.jks -storepass <STOREPASS> -keypass <KEYPASS> -alias dualpid \
          -keyalg RSA -keysize 4096 -validity 36500 -dname "CN=DualPID, OU=Dual, O=PID, L=Berlin, S=Berlin, C=DE"

Danach Datei `keystore.properties` anlegen (keine Anführungszeichen):

  storeFile=../keystore/dualpid-release.jks
  storePassword=<STOREPASS>
  keyAlias=dualpid
  keyPassword=<KEYPASS>

Sicherheit: Diese Dateien niemals in Git einchecken.
