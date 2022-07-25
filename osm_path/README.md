# What is this

This is a python package used to alter a given path (a .gpx file stored in **osm_path/gpx**) using data from osm maps.

Untraversable objects such as buildings, fences, trees or rivers are selected and the given path is changed to avoid and go around these objects.

Furthermore, roads are selected and the path tries to avoid these as well or if necessary cross them in the best manner possible (for now only a simple algorithm -- a more intelligent analysis is being implemented).

The output is the altered path and a figure visualising the situation (both saved to **osm_path/path**).

# How to run

-   Install requirements:

    **pip install -r ./requirements.txt**

-   Run the main file:

    **python3 ./main.py coords_file path_name**

-   For example:

    **python3 ./main.py cimicky.gpx cimicky_altered**

    (And you will find the altered path over at osm_path/path/cimicky_altered.gpx)