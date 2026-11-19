import numpy as np
import realtime
from dynamic_object_removal import RangeImageGhostFilter


class NearbyRangeImageGhostFilter(RangeImageGhostFilter):
    def filter(self, points, sensor_origin=(0.0, 0.0, 0.0)):
        _, keep_mask = super().filter(points, sensor_origin)
        # keep distant returns available as visibility evidence and map points
        keep_mask |= np.linalg.norm(points - sensor_origin, axis=1) > 3.0
        return points[keep_mask], keep_mask


if __name__ == '__main__':
    realtime.core.RangeImageGhostFilter = NearbyRangeImageGhostFilter
    raise SystemExit(realtime.main())
