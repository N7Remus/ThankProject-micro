import Adafruit_DHT


class DHT_class:

    # https://github.com/adafruit/Adafruit_Python_DHT alapján
    def __init__(self, pin):
        self.pin = pin
        self.temp = 0
        self.hum = 0

    def printPin(self):
        print("A DHT modul a " + str(self.pin) + " pinen kommunikál")

    def update(self):
        # Érzékelő típusának beállítása : DHT11,DHT22 vagy AM2302
        # A szenzorunk a következő GPIO-ra van kötve: self.pin, ezt inicializálásko elkérem
        # Ha a read_retry eljárást használjuk. Akkor akár 15x is megpróbálja kiolvasni az érzékelőből az adatot (és minden olvasás előtt 2 másodpercet vár).
        # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, gpio)
        humidity, temperature = Adafruit_DHT.read(Adafruit_DHT.DHT11, self.pin)
        # A DHT11 kiolvasása nagyon érzékeny az időzítésre és a Pi alkalmanként
        # nem tud helyes értéket kiolvasni. Ezért megnézzük, hogy helyesek-e a kiolvasott értékek.
        if humidity is not None and temperature is not None:
            self.temp = temperature
            self.hum = humidity

    def htmlFormat(self):
        content = "Hőmérséklet : " + str(self.temp) + " Páratartalom" + str(self.hum)
        return content
