test: face.mdl lex.py main.py matrix.py mdl.py display.py draw.py gmath.py yacc.py
	python main.py face.mdl

clean:
	rm *pyc *out parsetab.py *png *ppm *gif anim/*

clear:
	rm *pyc *out parsetab.py *ppm
