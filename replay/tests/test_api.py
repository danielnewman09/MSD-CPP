"""
API endpoint tests using FastAPI TestClient

Ticket: 0056d_fastapi_backend

Note: These tests require msd_reader to be built and available.
They will be skipped if msd_reader is not available.
"""

import pytest

try:
    import msd_reader

    MSD_READER_AVAILABLE = True
except ImportError:
    MSD_READER_AVAILABLE = False

if MSD_READER_AVAILABLE:
    from fastapi.testclient import TestClient
    from replay.app import app

pytestmark = pytest.mark.skipif(
    not MSD_READER_AVAILABLE, reason="msd_reader module not available"
)


@pytest.fixture
def client():
    """Create test client."""
    if MSD_READER_AVAILABLE:
        return TestClient(app)
    return None


def test_health_check(client):
    """Test health check endpoint."""
    if client is None:
        pytest.skip("msd_reader not available")

    response = client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_list_simulations(client):
    """Test listing available simulations."""
    if client is None:
        pytest.skip("msd_reader not available")

    response = client.get("/api/v1/simulations")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    # Content depends on recordings directory


def test_openapi_docs(client):
    """Test that OpenAPI docs are accessible."""
    if client is None:
        pytest.skip("msd_reader not available")

    response = client.get("/docs")
    assert response.status_code == 200

    response = client.get("/openapi.json")
    assert response.status_code == 200
    schema = response.json()
    assert "paths" in schema
    assert "/api/v1/simulations" in schema["paths"]


# Additional tests would require a test database fixture
# These are placeholder tests demonstrating the testing approach


@pytest.mark.skip(reason="Requires test database fixture")
def test_get_metadata(client):
    """Test getting simulation metadata."""
    sim_id = "test_simulation"
    response = client.get(f"/api/v1/simulations/{sim_id}/metadata")
    assert response.status_code == 200
    data = response.json()
    assert "bodies" in data
    assert "total_frames" in data


@pytest.mark.skip(reason="Requires test database fixture")
def test_get_frame_data(client):
    """Test getting frame data."""
    sim_id = "test_simulation"
    frame_id = 1
    response = client.get(
        f"/api/v1/simulations/{sim_id}/frames/{frame_id}/state"
    )
    assert response.status_code == 200
    data = response.json()
    assert "states" in data
    assert "collisions" in data
    assert "solver" in data


@pytest.mark.skip(reason="Requires test database fixture")
def test_bulk_frames(client):
    """Test bulk frame retrieval."""
    sim_id = "test_simulation"
    response = client.get(
        f"/api/v1/simulations/{sim_id}/frames/range?start=1&count=10"
    )
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    assert len(data) <= 10


@pytest.mark.skip(reason="Requires test database fixture")
def test_get_assets(client):
    """Test getting asset geometries."""
    sim_id = "test_simulation"
    response = client.get(f"/api/v1/simulations/{sim_id}/assets")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    assert all("positions" in asset for asset in data)
    assert all("vertex_count" in asset for asset in data)
