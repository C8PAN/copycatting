all:
	xelatex alter_ego.tex && bibtex alter_ego.aux && texindy alter_ego.idx && makeglossaries alter_ego && xelatex alter_ego.tex && xelatex alter_ego.tex

clean:
	find . -type f -iregex '.*\.\(aux\|log\|toc\|backup\|acr\|brf\|gz\|acn\|xdy\|alg\)$$'  -delete
