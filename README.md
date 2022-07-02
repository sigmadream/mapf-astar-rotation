# CBS base on A* with Rotation

## Requirements

- Python === `3.10.x`
- Numpy === `1.23.x`

## Run

```shell
// We recommend that you build a virtual environment.
$ pip install -r requirements.txt
$ python run.py
```

## Ref

- `A*`
    - [Hart, Peter E., Nils J. Nilsson, and Bertram Raphael. "A formal basis for the heuristic determination of minimum cost paths." IEEE transactions on Systems Science and Cybernetics 4.2 (1968): 100-107.](https://ieeexplore.ieee.org/iel5/4082035/4082123/04082128.pdf?casa_token=4nnDQ21EmjwAAAAA:aYVawKeQ3UZHglSwEVfd3cDAMBAB87-pKmw01rtpdRxQSk6no2CQup7RrrOj1GD7_duzLTBxd5o)
    - [Amit’s A\* Pages](http://theory.stanford.edu/~amitp/GameProgramming/)
    - [Korf, Richard E. "Recent progress in the design and analysis of admissible heuristic functions." International Symposium on Abstraction, Reformulation, and Approximation. Springer, Berlin, Heidelberg, 2000.](https://www.aaai.org/Papers/AAAI/2000/AAAI00-212.pdf)

- `CBS`(conflict based search)
    - [Boyarski, Eli, et al. "ICBS: Improved conflict-based search algorithm for multi-agent pathfinding." Twenty-Fourth International Joint Conference on Artificial Intelligence. 2015.](https://www.aaai.org/ocs/index.php/IJCAI/IJCAI15/paper/download/10955/10766)

- MAPF Benchmark Sets
    - [Multi-Agent Path-Finding (MAPF) Benchmarks](https://movingai.com/benchmarks/mapf.html)

## License

- MIT
