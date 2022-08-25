
ifneq ($V,1)
Q ?= @
endif

SRC	=	odroid_xu3.tex	\
		odroid_c1.tex	\
		odroid_c2.tex	\
		odroid_n1.tex	\
		odroid_n2.tex	\
		odroid_c4.tex   \
		bananapi_m5.tex \
		bananapi_m2s.tex \
		bananapi_cm4.tex

DVI	=	$(SRC:.tex=.dvi)

all:	$(DVI)
	$Q echo	Generating DVI

.tex.dvi:
	$Q latex $< $@
	$Q dvipdf $@

.PHONY:	clean
clean:
	$Q rm -f *.dvi *.aux *.log *.ps *.toc *.bak *~
