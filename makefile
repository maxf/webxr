watch:
	ls *.html *.js | entr -drs 'echo `date +%s%N | cut -b1-13` > timestamp.txt'

serve:
	http-server .
