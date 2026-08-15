import os

from sdformat_tools import sdf_util


def test_model_uri_to_urdf_uri_uses_package_uri(monkeypatch, tmp_path):
    prefix = tmp_path / 'install'
    model_root = prefix / 'share' / 'mesh_resources' / 'resource' / 'models'
    mesh_path = model_root / 'robot' / 'meshes' / 'body.dae'
    mesh_path.parent.mkdir(parents=True)
    mesh_path.touch()

    monkeypatch.setattr(sdf_util, 'sdf_paths', [str(model_root)])
    monkeypatch.setattr(
        sdf_util,
        'get_packages_with_prefixes',
        lambda: {'mesh_resources': str(prefix)},
    )

    assert sdf_util.model_uri_to_urdf_uri(
        'model://robot/meshes/body.dae'
    ) == 'package://mesh_resources/resource/models/robot/meshes/body.dae'


def test_model_uri_to_urdf_uri_falls_back_to_file_uri(monkeypatch, tmp_path):
    model_root = tmp_path / 'models'
    mesh_path = model_root / 'robot' / 'meshes' / 'body.dae'
    mesh_path.parent.mkdir(parents=True)
    mesh_path.touch()

    monkeypatch.setattr(sdf_util, 'sdf_paths', [str(model_root)])
    monkeypatch.setattr(sdf_util, 'get_packages_with_prefixes', lambda: {})

    assert sdf_util.model_uri_to_urdf_uri(
        'model://robot/meshes/body.dae'
    ) == 'file://' + os.path.abspath(mesh_path)


def test_model_uri_to_urdf_uri_returns_empty_for_unknown_model(monkeypatch):
    monkeypatch.setattr(sdf_util, 'sdf_paths', [])

    assert sdf_util.model_uri_to_urdf_uri(
        'model://missing/meshes/body.dae'
    ) == ''
