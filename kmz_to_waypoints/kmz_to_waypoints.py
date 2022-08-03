import zipfile
from fastkml import kml # https://fastkml.readthedocs.io/en/latest/usage_guide.html#read-a-kml-file-string
import shapely
import matplotlib.pyplot as plt

class KmzParser:
    def __init__(self,kml_fn,kmz_fn=None):
        self.kml_fn = kml_fn
        self.kmz_fn = kmz_fn

    def open_kmz(self):
        if not self.kmz_fn is None:
            self.kmz = zipfile.ZipFile(self.kmz_fn, 'r')
            self.kml_data = self.kmz.read(self.kml_fn)
        else:
            self.kmz = None
            with open(self.kml_fn, 'rt', encoding="utf-8") as f:
                self.kml_data = f.read()
    
    def extract_elements_from_kml(self):
        self.kml = kml.KML()
        self.kml.from_string(self.kml_data)

        self.data = list(self.kml.features())
        self.data = list(self.data[0].features())
        self.data = list(self.data[0].features())
    
    def get_lines(self):
        self.lines = []
        for element in self.data:
            if element.geometry.geom_type == "LineString":
                self.lines.append(element.geometry)
            if element.geometry.geom_type == "MultiLineString":
                for line in list(element.geometry.geoms):
                    self.lines.append(line)

    def visualize(self):
        fig,ax = plt.subplots()
        for line in self.lines:
            x,y = line.xy
            ax.plot(x,y)
        plt.show()
    
    def run(self):
        self.open_kmz()
        self.extract_elements_from_kml()
        self.get_lines()
        self.visualize()

if __name__ == "__main__":
    kmz_fn = "/home/adam/cras/kmz_to_waypoints/001_p.kmz"
    kml_fn = "doc.kml"
    parser = KmzParser(kml_fn = kml_fn, kmz_fn = kmz_fn)
    parser.run()

