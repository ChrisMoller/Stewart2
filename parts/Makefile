SOURCES = arm.scad \
          baseplate.scad \
          platform.scad \
          servobracket.scad \
          shaft.scad \
          topball.scad \
          topbracket.scad

STLS = $(SOURCES:.scad=.stl)
PNGS = $(SOURCES:.scad=.png)

%.stl:%.scad
	openscad -o $@ $<
	mv $@ STLs

%.png:%.scad
	openscad -o $@ $<
	mv $@ images

all:$(STLS)

images:$(PNGS)

clean:
	rm -rf STLs/*.stl images/*.png


