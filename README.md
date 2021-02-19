# Object-level fusion

[![CI](https://github.com/icaropires/objectlevel_fusion/actions/workflows/ci.yml/badge.svg)](https://github.com/icaropires/objectlevel_fusion/actions/workflows/ci.yml)

## Running the fusion layer

From the repository root, the fusion layer can be put in execution with just:

```bash
$ docker-compose up
```

After that, it will be waiting for messages of type `object_model_msgs/msg/ObjectModel` on the topic `/objectlevel_fusion/fusion_layer/fusion/submit`.

## Running tests

With the application up, tests can be run with:

```bash
docker-compose exec -T fusion_layer /entrypoint.sh colcon test --packages-above fusion_layer --event-handlers console_cohesion+
```
