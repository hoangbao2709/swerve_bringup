from types import SimpleNamespace

import pytest

from swerve_bridge.local_map_writer import write_trinary_map


def occupancy_grid(data, *, width=None, height=2, frame='map', resolution=0.05):
    width = len(data) // height if width is None else width
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=1.25, y=-2.5, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=2 ** -0.5, w=2 ** -0.5))),
        data=data,
    )


def pgm_raster(path):
    raw = path.read_bytes()
    header, raster = raw.split(b"255\n", 1)
    assert header.startswith(b"P5\n")
    return raster


def test_writes_nav2_compatible_trinary_pgm_yaml_and_geometry(tmp_path):
    grid = occupancy_grid([0, 100, 25, 50])

    result = write_trinary_map(grid, tmp_path / 'warehouse')

    assert (tmp_path / 'warehouse.pgm').is_file()
    assert result['width'] == 2 and result['height'] == 2
    assert result['known_cells'] == 4
    assert result['free_cells'] == 2
    assert result['occupied_cells'] == 1
    assert result['unknown_cells'] == 0
    # Nav2 writes the highest occupancy-grid row first; grayscale values match
    # Humble's trinary map_saver thresholds (free=254, unknown=205, occupied=0).
    assert pgm_raster(tmp_path / 'warehouse.pgm') == bytes((254, 205, 254, 0))
    yaml = (tmp_path / 'warehouse.yaml').read_text(encoding='utf-8')
    assert 'image: warehouse.pgm' in yaml
    assert 'mode: trinary' in yaml
    assert 'resolution: 0.05' in yaml
    assert 'origin: [1.25, -2.5, 1.57079632679]' in yaml
    assert 'free_thresh: 0.196' in yaml


def test_unknown_and_ambiguous_pixels_round_trip_as_unknown(tmp_path):
    grid = occupancy_grid([0, -1, 100, 50])

    write_trinary_map(grid, tmp_path / 'warehouse')

    # PGM is top-row first; Nav2 reverses rows when reconstructing OccupancyGrid.
    # Gray 205 must remain unknown with the YAML threshold, not turn into free.
    raster = pgm_raster(tmp_path / 'warehouse.pgm')
    image_rows = [raster[:2], raster[2:]]
    loaded_rows = list(reversed(image_rows))
    free_threshold = 0.196
    occupied_threshold = 0.65
    reconstructed = []
    for row in loaded_rows:
        for gray in row:
            occupancy = 1.0 - gray / 255.0
            reconstructed.append(100 if occupied_threshold < occupancy else
                                 0 if occupancy < free_threshold else -1)

    assert reconstructed == [0, -1, 100, -1]


@pytest.mark.parametrize('grid, message', [
    (occupancy_grid([0, 100, 0, 100], frame='odom'), "frame must be 'map'"),
    (occupancy_grid([-1, -1, -1, -1]), 'no known occupancy cells'),
    (occupancy_grid([0, 100, 0], width=2, height=2), 'do not match occupancy data'),
    (occupancy_grid([0, 100, 101, 0]), 'occupancy cells must be'),
    (occupancy_grid([0, 100, 0, 100], resolution=0), 'resolution must be positive'),
])
def test_invalid_map_is_rejected_without_artifacts(tmp_path, grid, message):
    with pytest.raises(ValueError, match=message):
        write_trinary_map(grid, tmp_path / 'invalid')
    assert not list(tmp_path.iterdir())


def test_existing_operator_files_are_never_overwritten(tmp_path):
    pgm = tmp_path / 'warehouse.pgm'
    yaml = tmp_path / 'warehouse.yaml'
    pgm.write_bytes(b'operator pgm')
    yaml.write_text('operator yaml', encoding='utf-8')

    with pytest.raises(FileExistsError, match='refusing to overwrite'):
        write_trinary_map(occupancy_grid([0, 100, 25, 50]), tmp_path / 'warehouse')

    assert pgm.read_bytes() == b'operator pgm'
    assert yaml.read_text(encoding='utf-8') == 'operator yaml'


def test_race_during_pair_commit_preserves_external_yaml_and_removes_own_pgm(
        tmp_path, monkeypatch):
    import swerve_bridge.local_map_writer as writer

    real_link = writer.os.link
    calls = 0

    def racing_link(source, destination):
        nonlocal calls
        calls += 1
        if calls == 2:
            (tmp_path / 'warehouse.yaml').write_text('external writer', encoding='utf-8')
        return real_link(source, destination)

    monkeypatch.setattr(writer.os, 'link', racing_link)
    with pytest.raises(FileExistsError):
        writer.write_trinary_map(occupancy_grid([0, 100, 25, 50]), tmp_path / 'warehouse')

    assert not (tmp_path / 'warehouse.pgm').exists()
    assert (tmp_path / 'warehouse.yaml').read_text(encoding='utf-8') == 'external writer'
    assert sorted(path.name for path in tmp_path.iterdir()) == ['warehouse.yaml']
