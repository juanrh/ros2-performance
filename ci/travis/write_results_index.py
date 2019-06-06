from jinja2 import Template
from pathlib import Path
from collections import namedtuple

_plot_width = {
  'benchmark_app_evaluation.svg': 1200,
  'cpu_ram_plot.svg': 1000
}

_results_index_template = Template('''
<!DOCTYPE html>
<html>
<head>
</head>
<body>

<h1>Results for benchmark run {{result_id}} (<a href="{{travis_build_url}}">Travis build {{travis_build_number}}</a>, <a href="{{travis_job_url}}">Travis job {{travis_job_number}}</a>)</h1>

{% for rmw_impl, benchmarks in rwm_benchmarks %}
  <h2>{{rmw_impl}} </h2>

  {% for benchmark in benchmarks %}
    <h3>{{benchmark.name}} (exit code=<font color="{{benchmark.color}}">{{benchmark.exit_code}}</font>)</h3>

    <div align="left">
      <a href="{{rmw_impl}}/{{benchmark.name}}/log.tgz">log.tgz</a>
    </div>

    {% for plot in plots %}
    <div align="left">
      <img src="{{rmw_impl}}/{{benchmark.name}}/plots/{{plot}}" alt="{{plot}}" width="{{plot_width[plot]}}">
    </div>
    {% endfor %}

  {% endfor %}

{% endfor %}

</body>
</html>
''')

BenchmarkInfo = namedtuple('BenchmarkInfo', ['name', 'path', 'exit_code', 'color'])
def _get_benchmark_info(benchmark_path):
    with open(str(benchmark_path / 'benchmark_exit_code.txt'), 'r') as in_f:
        exit_code = int(in_f.read())
    color = 'black' if exit_code == 0 else 'red'
    return BenchmarkInfo(name=benchmark_path.name, path=benchmark_path, exit_code=exit_code, color=color)


if __name__ == '__main__':
    import sys
    root_path = Path(sys.argv[1])
    [travis_build_number, travis_build_url, travis_job_number, travis_job_url] = sys.argv[2:]
    rmw_impls = [ p for p in root_path.iterdir() if p.is_dir() ]
    rwm_benchmarks = [
      (rmw_impl.name, [ _get_benchmark_info(p) for p in rmw_impl.iterdir() if p.is_dir() ])
        for rmw_impl in rmw_impls 
    ]
    index_path = root_path / 'index.html'
    with open(str(index_path), 'w') as out_f:
        rendered_index = _results_index_template.render(
          result_id=root_path.name,
          rwm_benchmarks=rwm_benchmarks,
          plots=[ p.name for p in (rwm_benchmarks[0][1][0].path / 'plots').iterdir() if p.is_file() ],
          plot_width=_plot_width,
          travis_build_number=travis_build_number,
          travis_build_url=travis_build_url,
          travis_job_number=travis_job_number,
          travis_job_url=travis_job_url
        )
        out_f.write(rendered_index)
