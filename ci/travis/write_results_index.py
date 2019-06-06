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

<h2>Results for benchmark run {{result_id}} (<a href="{{travis_build_url}}">Travis build {{travis_build_number}}</a>, <a href="{{travis_job_url}}">Travis job {{travis_job_number}}</a>)</h2>

{% for benchmark in benchmarks %}
  <h3>{{benchmark.name}} (exit code={{benchmark.exit_code}})</h3>

  <div align="left">
    <a href="{{benchmark.name}}/log.tgz">log.tgz</a>
  </div>

  {% for plot in plots %}
  <div align="left">
    <img src="{{benchmark.name}}/plots/{{plot}}" alt="{{plot}}" width="{{plot_width[plot]}}">
  </div>
  {% endfor %}
  
{% endfor %}

</body>
</html>
''')

BenchmarkInfo = namedtuple('BenchmarkInfo', ['name', 'path', 'exit_code'])
def _get_benchmark_info(benchmark_path):
    with open(str(benchmark_path / 'benchmark_exit_code.txt'), 'r') as in_f:
        exit_code = int(in_f.read())
    return BenchmarkInfo(name=benchmark_path.name, path=benchmark_path, exit_code=exit_code)


if __name__ == '__main__':
    import sys
    root_path = Path(sys.argv[1])
    [travis_build_number, travis_build_url, travis_job_number, travis_job_url] = sys.argv[2:]
    benchmarks = [ _get_benchmark_info(p) for p in root_path.iterdir() if p.is_dir() ]
    index_path = root_path / 'index.html'
    with open(str(index_path), 'w') as out_f:
        rendered_index = _results_index_template.render(
          result_id=root_path.name,
          benchmarks=benchmarks,
          plots=[ p.name for p in (benchmarks[0].path / 'plots').iterdir() if p.is_file() ],
          plot_width=_plot_width,
          travis_build_number=travis_build_number,
          travis_build_url=travis_build_url,
          travis_job_number=travis_job_number,
          travis_job_url=travis_job_url
        )
        out_f.write(rendered_index)
