### 1. `lane_map::parsePolygonFeatures`

- **Brief:** Changed existing function.
- **Location:** `/common/utils_map/include/utils/map/lane_map_parser.h`
- **Code:**
    ```cpp
    - PolygonFeatureList parsePolygonFeatures(const Json::Value& json_root, bool use_NED);
    + PolygonFeatureList parsePolygonFeatures(const Json::Value& json_root, bool use_NED, bool assert_polygon = true);
    ```

---

### 2. `lane_map::parsePointFeatures`

- **Brief:** New function.
- **Location:** `/common/utils_map/include/utils/map/lane_map_parser.h`
- **Code:**
    ```cpp
    + PointFeatureList parsePointFeatures(const Json::Value& json_root, bool use_NED, bool assert_point = true);
    ```
- **Description (based on implementation in `/common/utils_map/src/lane_map_parser.cpp`):**
    ```cpp
    /*
    * Given a JSON file, parse a list of Point features. The expected format is:
    * {
    *   "type": "FeatureCollection",
    *   "features": [
    *     {
    *       "type": "Feature",
    *       "properties": {
    *         "map_file": "somewhere_awesome.json",
    *         "name": "test",
    *         "id": "xxx"
    *       },
    *       "geometry": {
    *         "type": "Point",
    *         "coordinates": [ lat, long ]
    *       }
    *     }
    *   ]
    * }
    *
    * NOTE: Properties are not required but recommended for debugging and tracking purposes.
    */
    ```

---

### 3. `lane_map::LaneGroup` and `perception_msgs::msg::MapLaneGroup`

- **Brief:** Added a new boolean flag in `lane_map::LaneGroup` structure and `perception_msgs::msg::MapLaneGroup` message.
- **Location:**  
    - `/common/utils_map/include/utils/map/lane_map_structs.h`
    - `/home/ubuntu/frontier_ws/brain/msgs/perception_msgs/msg/MapLaneGroup.msg`
- **Code:**
    ```cpp
    bool is_left_shoulder_undrivable = false;
    ```
- **Description:**  
    - This flag likely indicates whether the left shoulder of a lane group is undrivable, similar to the existing `is_right_shoulder_undrivable` flag.

---

### 4. `lane_map::TerminalParkingSpot` and `perception_msgs::msg::TerminalParkingSpot`

- **Brief:** Introduced new structs for `lane_map::TerminalParkingSpot` and new `perception_msgs::msg::TerminalParkingSpot` message.
- **Location:**  
    - `/common/utils_map/include/utils/map/lane_map_structs.h`
    - `/msgs/perception_msgs/msg/TerminalParkingSpot.msg`
- **Code:**
    - Added `TerminalParkingSpot` struct:
      ```cpp
      struct TerminalParkingSpot {
          std::string parking_spot_id;
          BoostPolygon boundary_geometry;
          geometry_msgs::Point center_point;
          double heading;
          LaneGroupRef inflow_ref;
          LaneGroupRef outflow_ref;
          uint64_t left_boundary_id;
          uint64_t right_boundary_id;
      };
      ```
    - Added `TerminalParkingSpot.msg` struct:
      ```
      string parking_spot_id

      # Geography Properties
      geometry_msgs/Point[] boundary_geometry
      geometry_msgs/Point center_point
      float32 heading

      # Inflow/Outflow Properties
      MapLaneGroupRef inflow_ref
      MapLaneGroupRef outflow_ref
      ```
    - Added `TerminalParkingSpot.msg` into `TerminalMap.msg`:
      ```
      # Terminal Parking Boundaries
      TerminalParkingSpot[] terminal_parking_spots
      ```
- **Description:**  
    - The `TerminalParkingSpot` struct defines parking spots within a terminal, including their geometry, center point, heading, and lane group connections.

---

### 5. `lane_map::Pole`

- **Brief:** Introduced new struct for `lane_map::Pole`.
- **Location:** `/common/utils_map/include/utils/map/lane_map_structs.h`
- **Code:**
    ```cpp
    struct Pole {
        uint64_t id;
        double radius;
        geometry_msgs::Point center_point;

        // Temporary solution for lat/lon instead of UTM. 
        // TODO: Unify the coordinate system with parking spot.
        geometry_msgs::Point point;
    };
    ```
- **Description:**  
    - The `Pole` struct represents an object in the lane map, possibly for mapping utility poles or markers.

---

### 6. `lane_map_utils::getBoostPolygonFromPoints` and `lane_map_utils::getPointsFromBoostPolygon`

- **Brief:** Added two new functions for polygon-point conversion.
- **Location:** `/common/utils_map/include/utils/map/lane_map_utils.h`
- **Code:**
    ```cpp
    BoostPolygon getBoostPolygonFromPoints(const std::vector<geometry_msgs::Point>& points);
    std::vector<geometry_msgs::Point> getPointsFromBoostPolygon(const BoostPolygon& polygon);
    ```
- **Description:**  
    - `getBoostPolygonFromPoints`: Converts a set of points into a BoostPolygon.  
    - `getPointsFromBoostPolygon`: Extracts a set of points from a BoostPolygon.

---

### 7. `lane_map::ParkingSpotBoundary`

- **Brief:** Introduced new struct `lane_map::ParkingSpotBoundary`.
- **Location:** `/common/utils_map/include/utils/map/lane_map_structs.h`
- **Code:**
    ```cpp
    struct ParkingSpotBoundary {
        uint64_t boundary_id;
        BoostPolygon boundary_geometry;
    };
    ```
- **Description:**  
    - Defines the boundary of a parking spot, represented as a polygon.

---

### 8. `lane_map::TerminalMap` and `perception_msgs::msg::TerminalMap`

- **Brief:** Introduced `lane_map::TerminalMap` struct and `TerminalMap.msg`.
- **Location:** 
    - `/common/utils_map/include/utils/map/lane_map_structs.h`
    - `/msgs/perception_msgs/msg/TerminalMap.msg`
- **Code:**
    - TerminalMap structure
        ```cpp
        struct TerminalMap {
            std::vector<TerminalParkingSpot> terminal_parking_spots;
            std::vector<ParkingSpotBoundary> parking_boundaries;
        };
        ```
    - TerminalMap message
        ```
        Header header

        # frame this map is represented in
        MapFrame map_frame

        # Terminal Parking Boundaries
        TerminalParkingSpot[] terminal_parking_spots

        # Other Terminal Properties
        string terminal_id
        ```
- **Description:**  
    - Represents a collection of terminal parking spots and their boundaries.

---

### 9. `lane_map::PointFeature`

- **Brief:** Introduced `lane_map::PointFeature` struct.
- **Location:** `/common/utils_map/include/utils/map/lane_map_structs.h`
- **Code:**
    ```cpp
    struct PointFeature {
        uint64_t id;
        geometry_msgs::Point location;
        std::string feature_type;
    };
    ```
- **Description:**  
    - Represents a point-based feature in the lane map, identified by an ID, location, and type.

---

### 10. `LaneSubMap::insertFreeSpacePath`

- **Brief:** New function to insert a free space path into the submap.
- **Location:** `/mapping/maps/include/maps/lane_sub_map.h`
- **Code:**
    ```cpp
    std::optional<lane_map::LaneGroupRef> insertFreeSpacePath(const std::vector<Eigen::Vector3d>& path);
    ```
- **Description:**  
    - This function allows inserting a free space path represented by a vector of poses `(x, y, theta)`.  
    - If successful, it returns a reference to the created `LaneGroup`.

---

### 11. `maps::TiledZoneMap::getNearbyZoneTileId`

- **Brief:** New function to retrieve the closest zone tile.
- **Location:** `/mapping/maps/include/maps/tiled_zone_atlas.h`
- **Code:**
    ```cpp
    std::optional<uint64_t> getNearbyZoneTileId(const Eigen::Vector3d& position) const;
    ```
- **Description:**  
    - Given a position, this function returns the nearest zone tile ID, if available.



### 12. `enum class maps::MapZoneType`

- **Brief:** Introduced a new enumeration for map zone classification.
- **Location:** `/mapping/maps/include/maps/tiled_zone_atlas.h`
- **Code:**
    ```cpp
    enum class MapZoneType {
        ...,
        SHIFT_ZONES,
        HIGH_CURVATURE_ZONES,
        AGGRESSIVE_STEER_ZONES,
     +  TERMINAL_AREA_ZONES,
        TELEOP
    };
    ```
- **Description:**  
    - Defines different types of map zones, such as parking, pedestrian areas, restricted zones, and roadways.

---

### 13. `ego_lane_finder::getNominalLanes`

- **Brief:** New function to get nominal lanes from a tile.
- **Location:** `/mapping/maps/include/maps/utils/ego_lane_finder.h`
- **Code:**
    ```cpp
    std::vector<lane_map::LaneRef>
    getNominalLanes(const maps::LaneSubMap& map, 
                    const std::unordered_set<lane_map::LaneRef>& candidate_lanes,
                    lane_map_utils::TraverseDirection direction,
                    const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups = {});
    ```
- **Description:**  
    - Retrieves a list of lane references within a specific tile.

---

### 14. `lane_map_utils::getLaneRefsLeftToRight` and `lane_map_utils::getLanesLeftToRight`

- **Brief:** Functions to retrieve lane references and lanes from left to right.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    std::vector<lane_map::LaneRef> getLaneRefsLeftToRight(const lane_map::LaneGroupRef& lg_ref,
                                                          const maps::LaneSubMap& map);
    std::vector<lane_map::LaneRef> getLaneRefsLeftToRight(const lane_map::LaneGroup& lg);
    std::vector<const lane_map::Lane*> getLanesLeftToRight(const lane_map::LaneGroup& lg);
    ```
- **Description:**  
    - These functions return lane references and full lane objects ordered from left to right within a tile.

---

### 15. `lane_map_utils::getLaneRefsRightToLeft` and `lane_map_utils::getLanesRightToLeft`

- **Brief:** Functions to retrieve lane references and lanes from right to left.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    std::vector<lane_map::LaneRef> getLaneRefsRightToLeft(const lane_map::LaneGroupRef& lg_ref,
                                                          const maps::LaneSubMap& map);
    std::vector<lane_map::LaneRef> getLaneRefsRightToLeft(const lane_map::LaneGroup& lg);
    std::vector<const lane_map::Lane*> getLanesRightToLeft(const lane_map::LaneGroup& lg);
    ```
- **Description:**  
    - These functions return lane references and full lane objects ordered from right to left within a tile.

---

### 16. `lane_map_utils::isDrivingOnLeftShoulderAllowed`

- **Brief:** New function to check if driving on the left shoulder is allowed.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    bool isDrivingOnLeftShoulderAllowed(const maps::LaneSubMap& map, 
                                        const lane_map::LaneRef& lane_ref);
    ```
- **Description:**  
    - Returns `true` if driving on the left shoulder is permitted within a tile.

---

### 17. `lane_map_utils::backwardJunction` and `lane_map_utils::forwardJunction`

- **Brief:** New functions to retrieve backward and forward junctions.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    const lane_map::Junction* backwardJunction(const maps::LaneSubMap& map, 
                                               const lane_map::LaneRef lr);

    const lane_map::Junction* forwardJunction(const maps::LaneSubMap& map, 
                                              const lane_map::LaneRef lr);
    ```
- **Description:**  
    - `backwardJunction` returns the junction behind a given lane in a tile.  
    - `forwardJunction` returns the junction ahead of a given lane in a tile.

---

### 18. `lane_map_utils::ComputeInRouteScore`

- **Brief:** New function to compute a score for in-route evaluation.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    int ComputeInRouteScore(const lane_map::LaneRef& start_lr,
                            const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups,
                            const maps::LaneSubMap& map);
    ```
- **Description:**  
    - Computes a score evaluating the quality or validity of a given lane within a planned route.

---

### 19. `lane_map_utils::createLaneGroupFromFreeSpacePath`

- **Brief:** New function to create a lane group from a free space path.
- **Location:** `/mapping/maps/include/maps/utils/lane_map_utils.h`
- **Code:**
    ```cpp
    lane_map::LaneGroup createLaneGroupFromFreeSpacePath(const uint64_t tile_id, 
                                                         const std::vector<Eigen::Vector3d>& path);
    ```
- **Description:**  
    - This function attempts to construct a `LaneGroup` representing the path.

---

### 20. `/maps/utils/terminal_map_utils.h`

- **Brief:**: Introduce util functions for terminal maps.

- **Location**: `/mapping/maps/include/maps/utils/terminal_map_utils.h`

- **Code**:
    ```cpp
    /**
    * @brief Get the terminal parking boundaries for a given terminal.
    *
    * @param terminal_id The terminal ID.
    * @param map_dir The directory where the terminal map is stored.
    * @param localization The localization message.
    * @param frame_convention The frame convention.
    * @return std::unordered_map<std::string, lane_map::TerminalParkingSpot> The terminal parking boundaries.
    */
    std::unordered_map<std::string, lane_map::TerminalParkingSpot> getTerminalParkingBoundaries(
        const std::string& terminal_id, const std::string& map_dir, const perception_msgs::Localization& localization,
        maps::MapFrameType frame_convention = MapFrameType::GCS_NED);

    /**
    * @brief Get the terminal poles for a given terminal.
    *
    * @param terminal_id The terminal ID.
    * @param map_dir The directory where the terminal map is stored.
    * @param localization The localization message.
    * @param frame_convention The frame convention.
    * @return std::vector<lane_map::Pole> The terminal poles.
    */
    std::vector<lane_map::Pole> getTerminalPoles(const std::string& terminal_id, const std::string& map_dir,
                                                const perception_msgs::Localization& localization,
                                                maps::MapFrameType frame_convention = MapFrameType::GCS_NED);

    /**
    * @brief Get the terminal parking spot boundary lines for a given terminal.
    *
    * @param terminal_id The terminal ID.
    * @param map_dir The directory where the terminal map is stored.
    * @param localization The localization message.
    * @param frame_convention The frame convention.
    * @return std::unordered_map<uint64_t, lane_map::ParkingBoundary> The terminal poles.
    */
    std::unordered_map<uint64_t, lane_map::ParkingSpotBoundary> getTerminalParkingSpotBoundaries(
        const std::string& terminal_id, const std::string& map_dir, const perception_msgs::Localization& localization,
        maps::MapFrameType frame_convention = MapFrameType::GCS_NED);

    /**
    * @brief Get the terminal map for a given terminal.
    *
    * @param terminal_id The terminal ID.
    * @param map_dir The directory where the terminal map is stored.
    * @param localization The localization message.
    * @param frame_convention The frame convention.
    * @return lane_map::TerminalMap The terminal map.
    */
    lane_map::TerminalMap getTerminalMap(const std::string& terminal_id, const std::string& map_dir,
                                        const perception_msgs::Localization& localization,
                                        maps::MapFrameType frame_convention = MapFrameType::GCS_NED);

    lane_map::TerminalMap getTerminalMap(const std::string& terminal_id, const std::string& map_dir, const double latitutde,
                                        const double longitude,
                                        maps::MapFrameType frame_convention = MapFrameType::GCS_NED);

    lane_map::TerminalMap buildTerminalMap(const perception_msgs::TerminalMap& terminal_map_msg);
    ```

- **Description**: TerminalMap seems to be a different map format than common map format. It is designed just for terminal features.

---

### 21. `/maps/utils/route_generation_node.py`

- **Location**: `/mapping/maps/include/maps/utils/terminal_map_utils.h`

- **Brief**: 
    1. Handling Terminal Parking Spots**
        - **New parameters:**
            - `self.terminal_parking_handoff_offset`: Default value set to `30.0` meters.
            - `self.terminal_stop_line_offset`: Default value set to `4.0` meters.

        - **Added methods:**
            - `_read_terminal_metadata()`: Reads terminal parking spot references from ROS parameters.
            - `_add_terminal_waypoint()`: Inserts terminal waypoints based on inflow/outflow properties of parking spots.
            - `_add_parking_spot_waypoint()`: Converts parking spot data into route waypoints.

        - **Functionality:**
            - Reads terminal parking spot references (`MapParkingSpotRef`).
            - Loads parking spot data from the `free_space_dir`.
            - Inserts waypoints corresponding to inflow (entry) and outflow (exit) of terminal parking.
            - Ensures vehicles stop at designated terminal waypoints.
            - Updates `route_msg` with parking spot waypoints.

    - 2. Stop Line Waypoints Integration**
        - **Added method:**
            - `_add_stop_line_waypoints()`: Identifies and adds stop-line waypoints along the planned route.

        - **Functionality:**
            - Retrieves stop-line waypoints using `routing_utils.get_waypoints_with_stop_line()`.
            - Projects stop-line waypoints forward to ensure vehicles stop at the correct location.
            - Updates geometry of waypoints for better alignment with actual stop-line positions.

    - 3. Route Calculation Enhancements**
        - Updated `find_route()` call to use `self.lane_tile_dir` instead of a hardcoded tile directory.
        - Ensures terminal parking information is correctly included in the `route_msg`.

---

### 22. `perception_msgs::msg::MapParkingSpotRef` and `perception_msgs::msg::MapTrip`

- **Brief**: Add a new message `MapParkingSpotRef.msg` and add it into `MapTrip.msg`.
- **Location**:
    - `/msgs/perception_msgs/msg/MapParkingSpotRef.msg`
    - `/msgs/perception_msgs/msg/MapTrip.msg`
- **Code**:
    - New `MapParkingSpotRef.msg`:
        ```cpp
        string terminal_id
        string id
        ```
    - Added in `MapTrip.msg`:
        ```cpp
        # Only used for terminal-to-terminal driving, null otherwise
        MapParkingSpotRef origin_parking_spot
        MapParkingSpotRef destination_parking_spot
        ```
- **Description**:  
    - The `MapParkingSpotRef` message represents a reference to a parking spot within a terminal.  
    - This addition allows for explicit tracking of terminal parking spots in `MapTrip` messages.

---

### 23. `perception_msgs::msg::MapWaypoint`

- **Brief**: Added new waypoint types and a heading field.
- **Location**: `/msgs/perception_msgs/msg/MapWaypoint.msg`
- **Code**:
    ```cpp
    uint8 WAYPOINT_PARKING_INFLOW = 8
    uint8 WAYPOINT_PARKING_OUTFLOW = 9
    uint8 WAYPOINT_PARKING_SPOT = 10
    uint8 WAYPOINT_STOP_LINE = 11

    uint8 waypoint_type
    float64 heading
    string description
    string source
    ```
- **Description**:  
    - Introduces new waypoint types related to parking and stop-line locations.
    - Adds a `heading` field to store the orientation of the waypoint.

---

### 24. `perception_msgs::msg::Object`

- **Brief**: Added new object types related to construction elements.
- **Location**: `/msgs/perception_msgs/msg/Object.msg`
- **Code**:
    ```cpp
    uint8 SPEED_LIMIT_SIGN = 10
    uint8 CONSTRUCTION_SIGN = 11
    uint8 CONSTRUCTION_CONE = 12
    uint8 CONSTRUCTION_DELINEATOR = 13
    uint8 CONSTRUCTION_BARREL = 14
    ```
- **Description**:  
    - Adds specific construction-related objects such as cones, delineators, and barrels.
    - Enables better classification of objects detected in a construction zone.

---

### 25. `perception_msgs::msg::PoleDetection`

- **Brief**: Added geographic coordinates for pole detection.
- **Location**: `/msgs/perception_msgs/msg/PoleDetection.msg`
- **Code**:
    ```cpp
    # WGS84 of associated pole on map
    float64 latitude
    float64 longitude
    ```
- **Description**:  
    - Adds latitude and longitude fields to associate detected poles with map locations.

---

### 26. `perception_msgs::msg::Scene`

- **Brief**: Added new terminal area zone field and terminal map.
- **Location**: `/msgs/perception_msgs/msg/Scene.msg`
- **Code**:
    ```cpp
    bool in_terminal_area_zone
    TerminalMap terminal_map
    ```
- **Description**:  
    - Adds a flag to indicate if the scene is within a terminal area.
    - Introduces a `TerminalMap` field to store terminal-related features.

---

### 27. `perception_msgs::msg::TrackedObjectState`

- **Brief**: Added new tracked object types.
- **Location**: `/msgs/perception_msgs/msg/TrackedObjectState.msg`
- **Code**:
    ```cpp
    uint8 SPEED_LIMIT_SIGN = 4
    uint8 CONSTRUCTION = 5
    uint8 DEBRIS = 6
    ```
- **Description**:  
    - Expands tracked object classifications to include speed limit signs, construction elements, and debris.

---

### 28. `planning_msgs::msg::PlannerInfo`

- **Brief**: Added new terminal state field and driving mode classifications.
- **Location**: `/msgs/planning_msgs/msg/PlannerInfo.msg`
- **Code**:
    ```cpp
    int32 SOURCE = 0
    int32 HIGHWAY_DRIVING = 1
    int32 TERMINAL_LANE_FOLLOWING = 2
    int32 PARK_HANDOFF = 3
    int32 PARK = 4
    int32 UNPARK = 5
    int32 SINK = 6

    int32 terminal_state
    ```
- **Description**:  
    - Introduces `terminal_state`, which categorizes the vehicle’s state in a terminal environment.
    - Defines new planner states for different driving scenarios such as highway driving, parking, and lane following in a terminal.

---

### 29. `/terminal_state_machine/abstract_terminal_state.h`

- **Brief:**: Abstract terminal state. This class is an abstract class that represents a state in the terminal state machine. All such states have access to the latest external signals, the waypoints, and the terminal state.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/abstract_terminal_state.h`

- **Code**:
    ```cpp
    class AbstractTerminalState : public FSMState
    {
    public:
    /**
    * @brief Constructor
    *
    * @param id ID of the state
    * @param name Name of the state
    * @param external_signals External signals that the terminal state machine needs to receive
    * @param waypoints Waypoints that mark the places in which we transition between planners
    * @param terminal_state Internal state of the terminal state machine
    */
    AbstractTerminalState(int id, const std::string& name, const ExternalSignals& external_signals,
                            const std::vector<Waypoint>& waypoints, TerminalState& terminal_state);

    int Update() override = 0;

    /**
    * @brief Get the signed distance to the currently targeting waypoint
    *
    * The distance will be positive if the waypoint is ahead of ego and negative if it is behind.
    *
    * @return Distance to the current waypoint
    */
    double DistanceToCurrentWaypoint() const;

    /**
    * @brief True if the vehicle is stopped
    */
    bool Stopped() const;

    public:
    const ExternalSignals& external_signals_; // External signals that the terminal state machine needs to receive
    const std::vector<Waypoint>& waypoints_;  //  Waypoints that mark the places in which we transition between planners
    TerminalState& terminal_state_;           // Internal state of the terminal state machine
    };
    ```

### 30. `/terminal_state_machine/highway_driving.h`

- **Brief:** Highway driving state. This state represents the truck's default behaviors on the highway, including lane following, merging, and lane changing.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/highway_driving.h`

- **Code**:
    ```cpp
    class HighwayDriving : public AbstractTerminalState
    {
    public:
        HighwayDriving(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                       TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            OUT_OF_HIGHWAY,
            COMPLETED,
        };
    };
    ```

---

### 31. `/terminal_state_machine/highway_sink.h`

- **Brief:** Sink state. This state represents the final state in the terminal state machine.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/highway_sink.h`

- **Code**:
    ```cpp
    class HighwaySink : public AbstractTerminalState
    {
    public:
        HighwaySink(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                    TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
        };
    };
    ```

---

### 32. `/terminal_state_machine/park.h`

- **Brief:** Park state. When this state is active, the truck is parking using the free space planner.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/park.h`

- **Code**:
    ```cpp
    class Park : public AbstractTerminalState
    {
    public:
        Park(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
             TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            PARKED,
        };
    };
    ```

---

### 33. `/terminal_state_machine/park_handoff.h`

- **Brief:** Park handoff state. This state is responsible for handling the transition between lane-based planning and free-space planning.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/park_handoff.h`

- **Code**:
    ```cpp
    class ParkHandoff : public AbstractTerminalState
    {
    public:
        /**
         * @brief Constructor
         *
         * @param id ID of the state
         * @param external_signals External signals that the terminal state machine needs to receive
         * @param waypoints Waypoints that mark the places in which we transition between planners
         * @param terminal_state Internal state of the terminal state machine
         */
        ParkHandoff(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                    TerminalState& terminal_state);

        // Needed to update the handoff state variable
        void OnExit() override;

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            READY_TO_HANDOFF,
        };
    };
    ```

---

### 34. `/terminal_state_machine/source.h`

- **Brief:** Source state. This state represents the entry point into the terminal state machine, handling FSM initialization and allowing entry from different locations.

- **Location**: `/planning/terminal_state_machine/include/terminal_state_machine/source.h`

- **Code**:
    ```cpp
    class Source : public AbstractTerminalState
    {
    public:
        Source(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
               TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            TRANSITION_TO_LANE_FOLLOWING,
            TRANSITION_TO_UNPARK,
            TRANSITION_TO_HIGHWAY,
        };
    };
    ```

---

### 35. `terminal_lane_following.h`

- **Brief:** Lane following state. This state enables the truck to follow lanes within the terminal while handling merges and stop lines.
- **Location:** `/planning/terminal_state_machine/include/terminal_state_machine/terminal_lane_following.h`
- **Code:**
    ```cpp
    class TerminalLaneFollowing : public AbstractTerminalState
    {
    public:
        TerminalLaneFollowing(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                              TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            REACHED_HANDOFF,
            OUT_OF_TERMINAL,
        };
    };
    ```

---

### 36. `terminal_sink.h`

- **Brief:** Sink state. This state represents the final state in the terminal state machine.
- **Location:** `/planning/terminal_state_machine/include/terminal_state_machine/terminal_sink.h`
- **Code:**
    ```cpp
    class TerminalSink : public AbstractTerminalState
    {
    public:
        TerminalSink(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                     TerminalState& terminal_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
        };
    };
    ```

---

### 37. `terminal_state_machine.h`

- **Brief:** Terminal state machine. This class manages transitions between different planners inside the terminal.
- **Location:** `/planning/terminal_state_machine/include/terminal_state_machine/terminal_state_machine.h`
- **Code:**
    ```cpp
    struct TerminalState
    {
        bool parked{ false };
        bool at_handoff{ false };
        int current_waypoint{ 0 };
    };

    struct ExternalSignals
    {
        bool in_terminal;
        double speed;
        double travel;
        bool free_space_plan_ready;
    };

    class TerminalStateMachine
    {
    public:
        TerminalStateMachine(std::vector<Waypoint> waypoints, TerminalState terminal_state);
        std::string CurrentStateName() const;
        int CurrentStateID() const;
        bool ShouldDecelerateToStop() const;
        TerminalState Update(const ExternalSignals& external_signals);

    private:
        std::vector<Waypoint> waypoints_;
        ExternalSignals external_signals_;
        TerminalState terminal_state_;
        FiniteStateMachine state_machine_;
    };
    ```

---

### 38. `unpark.h`

- **Brief:** Unpark state. When this state is active, the truck is unparking using the free space planner.
- **Location:** `/planning/terminal_state_machine/include/terminal_state_machine/unpark.h`
- **Code:**
    ```cpp
    class Unpark : public AbstractTerminalState
    {
    public:
        Unpark(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
               TerminalState& terminal_state);

        void OnEnter() override;
        void OnExit() override;
        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            AT_HANDOFF,
        };
    };
    ```

---

### 39. `abstract_terminal_stop_go_state.h`

- **Brief:** Abstract terminal stop-go state. This class represents a state in the stop-go terminal state machine, allowing access to external signals and waypoints.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/abstract_terminal_stop_go_state.h`
- **Code:**
    ```cpp
    class AbstractState : public FSMState
    {
    public:
        AbstractState(int id, const std::string& name, const ExternalSignals& external_signals,
                      const std::vector<Waypoint>& waypoints, TerminalState& state);

        int Update() override = 0;
        bool EgoStopped() const;
        double DistanceToCurrentWaypoint() const;
        bool AtRouteEnd() const;

    protected:
        const ExternalSignals& external_signals_;
        const std::vector<Waypoint>& waypoints_;
        TerminalState& terminal_stop_go_state_;
    };
    ```

---

### 40. `driving.h`

- **Brief:** Driving state. This state enables the truck to drive within the stop-go terminal planning framework.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/driving.h`
- **Code:**
    ```cpp
    class Driving : public AbstractState
    {
    public:
        Driving(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                TerminalState& terminal_stop_go_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            TRANSITION_TO_STOPPED,
            TRANSITION_TO_END,
        };
    };
    ```

---

### 41. `sink.h`

- **Brief:** Sink state. This state represents the final state in the terminal stop-go state machine.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/sink.h`
- **Code:**
    ```cpp
    class Sink : public AbstractState
    {
    public:
        Sink(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
             TerminalState& terminal_stop_go_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
        };
    };
    ```

---

### 42. `source.h`

- **Brief:** Source state. This state represents the entry point into the stop-go terminal state machine.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/source.h`
- **Code:**
    ```cpp
    class Source : public AbstractState
    {
    public:
        Source(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
               TerminalState& terminal_stop_go_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            TRANSITION_TO_DRIVING,
        };
    };
    ```

---

### 43. `stopped.h`

- **Brief:** Stopped state. This state is responsible for managing the truck's stopped condition within the terminal stop-go state machine.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/stopped.h`
- **Code:**
    ```cpp
    class Stopped : public AbstractState
    {
    public:
        Stopped(int id, const ExternalSignals& external_signals, const std::vector<Waypoint>& waypoints,
                TerminalState& terminal_stop_go_state);

        int Update() override;

        // Outward endpoints
        enum
        {
            SELF_TRANSITION,
            TRANSITION_TO_DRIVING,
        };
    };
    ```

---

### 44. `terminal_stop_go_state_machine.h`

- **Brief:** Terminal stop-go state machine. This state machine manages transitions between stopping and driving at stop lines in terminal areas.
- **Location:** `/planning/terminal_state_machine/include/terminal_stop_go_state_machine/terminal_stop_go_state_machine.h`
- **Code:**
    ```cpp
    class TerminalStopGoStateMachine
    {
    public:
        TerminalStopGoStateMachine(std::vector<Waypoint> waypoints, TerminalState terminal_stop_go_state);
        std::string CurrentStateName() const;
        int CurrentStateID() const;
        int CurrentWaypointId() const;
        TerminalState Update(const ExternalSignals& external_signals);

    private:
        std::vector<Waypoint> waypoints_;
        ExternalSignals external_signals_;
        TerminalState terminal_stop_go_state_;
        FiniteStateMachine state_machine_;
    };
    ```

---

### 45. `planning::action_utils::buildTerminalParkingActionTarget`

- **Brief:** New function to generate `ActionTarget`.
- **Location:** `/planning/behavior_planner/include/behavior_planner/action_target_helpers.h`
- **Code:**
    ```cpp
    ActionTarget buildTerminalParkingActionTarget(const BehaviorType& behavior_origin, const SpeedLimitProfile& speed_limit,
                                                const PredictedScene& scene, const PlannerParameters& params,
                                                const ReferenceLine& reference_line,
                                                nominal_tg::PathIntegralNominal& path_integral_nominal_generator);
    ```
- **Description:**  
    - This function sets up an action target with all needed fields for terminal parking.

---

### 46. `/behavior_planner/behaviors/mock_behavior_base.h`

- **Brief:** Mock behavior base class for unit testing. This class provides a mock implementation of `BehaviorBase` for testing behavior planner functionalities.
- **Location:** `/planning/behavior_planner/include/behavior_planner/behaviors/mock_behavior_base.h`
- **Code:**
    ```cpp
    class MockBehaviorBase : public BehaviorBase
    {
    public:
        MOCK_METHOD(BehaviorType, type, (), (const));

        MOCK_METHOD(void, setParams, (const PlannerParameters& params), ());

        MOCK_METHOD(std::optional<ActionTarget>, generateAction,
                    (const std::shared_ptr<const PredictedScene>& scene,
                     const SituationConstraints& situation_constraints,
                     BehaviorPlannerDebugInfo& debug_info),
                    ());
    };
    ```
- **Description:**  
    - Uses Google Mock (`gmock`) to define a mock class for `BehaviorBase`.  
    - Provides mock methods for behavior type retrieval, parameter setting, and action generation.  
    - Facilitates unit testing by allowing controlled behavior simulation in test cases.

---

### 47. `/behavior_planner/behaviors/terminal_parking/terminal_parking_behavior.h`

- **Brief:** Terminal parking behavior. This class is responsible for generating a parking path in terminal areas.
- **Location:** `/planning/behavior_planner/include/behavior_planner/behaviors/terminal_parking/terminal_parking_behavior.h`
- **Code:**
    ```cpp
    class TerminalParkingBehavior : public BehaviorBase
    {
    public:
        TerminalParkingBehavior();

        BehaviorType type() const final
        {
            return BehaviorType::FREE_SPACE;
        }

        void setParams(const PlannerParameters& planner_params);
        void computeTerminalParkingPath(const SituationConstraints& situation_constraints, const PredictedScene& scene);
        bool isTerminalParkingPathReady() const;
        std::optional<std::vector<Eigen::Vector3d>> getTerminalParkingPathInUtm() const;
        void Clear();

        std::optional<ActionTarget> generateAction(const std::shared_ptr<const PredictedScene>& scene,
                                                   const SituationConstraints& situation_constraints,
                                                   BehaviorPlannerDebugInfo& debug_info);

    private:
        std::optional<std::vector<Pose2D>> parking_path_;
        std::optional<Eigen::Vector3d> parking_path_frame_origin_;
        PlannerParameters planner_params_;
        nominal_tg::PathIntegralNominal path_integral_nominal_generator_;
        FreeSpacePlannerParameters free_space_planner_params_;
        EgoFootprint ego_footprint_;
        TerminalPlannerParams terminal_planner_params_;
        std::optional<FreeSpacePlanner> free_space_planner_;

        std::optional<Pose2D> getParkingSpotPoseInEgoFrame(const PredictedScene& scene) const;
    };
    ```
- **Description:**  
    - Implements terminal parking using free-space planning techniques.  
    - Computes a parking path while considering constraints and environment conditions.  
    - Integrates with `FreeSpacePlanner` to generate feasible trajectories.  
    - Provides a method to check if a valid parking path is ready.

---

### 48. `/behavior_planner/behaviors/terminal_parking/terminal_unparking_behavior.h`

- **Brief:** Terminal unparking behavior. This class is responsible for generating an unparking path in terminal areas.
- **Location:** `/planning/behavior_planner/include/behavior_planner/behaviors/terminal_parking/terminal_unparking_behavior.h`
- **Code:**
    ```cpp
    class TerminalUnparkingBehavior : public BehaviorBase
    {
    public:
        TerminalUnparkingBehavior();

        BehaviorType type() const final
        {
            return BehaviorType::FREE_SPACE;
        }

        void setParams(const PlannerParameters& planner_params);
        void computeTerminalUnparkingPath(const SituationConstraints& situation_constraints, const PredictedScene& scene);
        bool isTerminalUnparkingPathReady() const;
        std::optional<std::vector<Eigen::Vector3d>> getTerminalUnparkingPathInUtm() const;
        void Clear();

        std::optional<ActionTarget> generateAction(const std::shared_ptr<const PredictedScene>& scene,
                                                   const SituationConstraints& situation_constraints,
                                                   BehaviorPlannerDebugInfo& debug_info);

    private:
        std::optional<std::vector<Pose2D>> unparking_path_;
        std::optional<Eigen::Vector3d> unparking_path_frame_origin_;
        PlannerParameters planner_params_;
        nominal_tg::PathIntegralNominal path_integral_nominal_generator_;
        FreeSpacePlannerParameters free_space_planner_params_;
        EgoFootprint ego_footprint_;
        TerminalPlannerParams terminal_planner_params_;
        std::optional<FreeSpacePlanner> free_space_planner_;

        std::optional<Pose2D> getUnparkHandoffPoseInEgoFrame(const PredictedScene& scene) const;
    };
    ```
- **Description:**  
    - Implements terminal unparking using free-space planning.  
    - Computes an exit path from parking spots while considering constraints and vehicle position.  
    - Integrates with `FreeSpacePlanner` for trajectory generation.  
    - Ensures smooth transition from parking to lane-based driving.

---

### 49. `/behavior_planner/terminal_planner.h`

- **Brief:** Complete the `TerminalPlanner` class. The main realization of terminal features.
- **Location:** `/planning/behavior_planner/include/behavior_planner/terminal_planner.h`
- **Code:**
    ```cpp
    class TerminalPlanner
    {
    public:
    TerminalPlanner(ros::NodeHandle* nh, std::shared_ptr<DriverAlerts> driver_alerts,
                    std::shared_ptr<TruckFunctions> truck_functions_interface);

    /**
    * This constructor is used for testing purposes only
    */
    TerminalPlanner(ros::NodeHandle* nh, std::shared_ptr<DriverAlerts> driver_alerts,
                    std::shared_ptr<TruckFunctions> truck_functions_interface, BehaviorBase& follow_lane_behavior,
                    BehaviorBase& parking_behavior, BehaviorBase& unparking_behavior);

    /**
    * @brief Sets all parameters needed
    */
    void setParams(const PlannerParameters& params);

    /**
    * @brief Resets internal state
    */
    void reset();

    /**
    * @brief Update the current behavior actions we want to send to trajectory
    * generation
    *
    * @param[in] situation_constraints The situation constraints
    * @param[in] scene The current scene
    * @param[in] operational_domain The operational domain
    * @param[out] debug_info The debug info
    */
    std::unordered_map<BehaviorType, ActionTarget> planActionTargets(const SituationConstraints& situation_constraints,
                                                                    const std::shared_ptr<const PredictedScene>& scene,
                                                                    const OperationalDomain& operational_domain,
                                                                    BehaviorPlannerDebugInfo* debug_info,
                                                                    bool& free_space_plan_ready);

    /**
    * @brief Get the free space path, if it exists
    *
    * @return std::optional<std::vector<Eigen::Vector2d>> The free space path
    */
    std::optional<std::vector<Eigen::Vector3d>> getFreeSpacePath() const;


    private:
    // Parameters
    PlannerParameters planner_params_;

    // Base behaviors that allows for unit testing
    BehaviorBase& follow_lane_behavior_;
    BehaviorBase& parking_behavior_;
    BehaviorBase& unparking_behavior_;

    // Action handlers for terminal planning
    TerminalFollowLaneBehavior default_follow_lane_behavior_;
    TerminalParkingBehavior default_parking_behavior_;
    TerminalUnparkingBehavior default_unparking_behavior_;

    std::future<void> compute_parking_path_future_;
    bool parking_path_compute_in_progress_ = false;
    bool unparking_path_ready_ = false;
    };
    ```

---

### 50. `FreeSpacePlanner.cfg`

- **Brief:** New configuration file for free space planner.
- **Location:** `/planning/planning_common/cfg/FreeSpacePlanner.cfg`
- **Code:**
    ```py
    PACKAGE = "planner"

    gen = ParameterGenerator()

    gen.add("turning_radius", double_t, 0,
            "turning radius of the vehicle [meters]",
            10.6, 6.0, 12.0)
    gen.add("transition_cost", double_t, 0,
            "cost of transitioning between different maneuvers",
            0.1, 0.0, 1.0)
    gen.add("iterations", int_t, 0,
            "number of iterations for the piano mover planner",
            2, 1, 5)
    gen.add("search_radius", double_t, 0,
            "search radius to use for improved goal search [meters]",
            1.0, 0.5, 5.0)
    gen.add("cell_size", double_t, 0,
            "size of a cell in the costmap [meters]",
            0.2, 0.01, 0.5)
    gen.add("space_dim_size", int_t, 0,
            "size of the space dimension in the costmap",
            256, 64, 1024)
    gen.add("angle_dim_size", int_t, 0,
            "size of the angle dimension in the costmap",
            256, 64, 1024)
    gen.add("weight1", double_t, 0,
            "weight for the distance cost in the goal search algorithm",
            2.0, 0.0, 10.0)
    gen.add("weight2", double_t, 0,
            "weight for the heading cost in the goal search algorithm",
            2.0, 0.0, 10.0)

    exit(gen.generate(PACKAGE, "planner", "FreeSpacePlanner"))
    ```

---

### 51. `TerminalPlanner.cfg`

- **Brief:** New configuration file for terminal planner.
- **Location:** `/planning/planning_common/cfg/TerminalPlanner.cfg`
- **Code:**
    ```py
    PACKAGE = "planner"

    gen = ParameterGenerator()

    gen.add("transform_min_cost", double_t, 0,
            "Minimum cost for the distance transform",
            0.1, 0.0, 1.0)
    gen.add("transform_max_cost_parking_spot", double_t, 0,
            "Maximum cost for the distance transform for parking spot",
            1.0, 0.0, 10.0)
    gen.add("transform_max_cost_static_obstacle", double_t, 0,
            "Maximum cost for the distance transform for static obstacle",
            2.0, 0.0, 10.0)
    gen.add("transform_scale_cost", double_t, 0,
            "Scale for the distance transform",
            2.0, 0.0, 10.0)
    gen.add("costmap_longitudinal_margin", double_t, 0,
            "Longitudinal Offset in CostMap2D [meters]",
            1.0, 0.0, 10.0)
    gen.add("free_space_path_extend_distance", double_t, 0,
            "Extension distance of the free space path [meters]",
            10.0, 0.0, 100.0)
    gen.add("free_space_path_extend_delta_distance", double_t, 0,
            "Distance between each point for extension of the free space path [meters]",
            0.2, 0.1, 1.0)

    exit(gen.generate(PACKAGE, "planner", "TerminalPlanner"))
    ```

---

### 52. `planning::BehaviorSpeedConstraints`

- **Brief:** Modify enumeration based on `BehaviorType` update.
- **Location:** `/planning/planning_common/include/planning_common/types/constraint_map.h`
- **Code:**
    ```cpp
    BehaviorSpeedConstraints()
    {
    -    for (auto behavior_type : { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND,
    -                                BehaviorType::MERGE, BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE,
    -                                BehaviorType::TERMINAL_FOLLOW_LANE, BehaviorType::TERMINAL_PARK, BehaviorType::TERMINAL_UNPARK }) {
    +    for (auto behavior_type :
    +         { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND, BehaviorType::MERGE,
    +           BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE, BehaviorType::FREE_SPACE }) {
        constraints[behavior_type] = SpeedLimitProfile();
        }
    }

    void setLimitingType(const SpeedLimiterType& limiter_type)
    {
    -    for (auto behavior_type : { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND,
    -                                BehaviorType::MERGE, BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE,
    -                                BehaviorType::TERMINAL_FOLLOW_LANE, BehaviorType::TERMINAL_PARK, BehaviorType::TERMINAL_UNPARK }) {
    +    for (auto behavior_type :
    +         { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND, BehaviorType::MERGE,
    +           BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE, BehaviorType::FREE_SPACE }) {
        constraints[behavior_type].limiting_type = limiter_type;
        }
    }

    void setAllBehaviorConstraints(const double speed_constraint)
    {
    -    for (auto behavior_type : { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND,
    -                                BehaviorType::MERGE, BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE,
    -                                BehaviorType::TERMINAL_FOLLOW_LANE, BehaviorType::TERMINAL_PARK, BehaviorType::TERMINAL_UNPARK }) {
    +    for (auto behavior_type :
    +         { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND, BehaviorType::MERGE,
    +           BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE, BehaviorType::FREE_SPACE }) {
        constraints[behavior_type].setConstantSpeedLimit(speed_constraint);
        }
    }

    void setAllBehaviorConstraints(const PiecewiseLinear& speed_limit)
    {
    -    for (auto behavior_type : { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND,
    -                                BehaviorType::MERGE, BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE,
    -                                BehaviorType::TERMINAL_FOLLOW_LANE, BehaviorType::TERMINAL_PARK, BehaviorType::TERMINAL_UNPARK }) {
    +    for (auto behavior_type :
    +         { BehaviorType::FOLLOW_ROAD, BehaviorType::TELEOP, BehaviorType::GAP_FIND, BehaviorType::MERGE,
    +           BehaviorType::OVERTAKE, BehaviorType::YIELD_TO_LANE_CHANGE, BehaviorType::FREE_SPACE }) {
        constraints[behavior_type].speed_limit = speed_limit;
        }
    }
    ```

- **Description:**
    - If add new `BehaviorType`, here is the place to apply constraints on it.

---


### 53. `planning::PlannerParameters`

- **Brief:** Modify parameter loading function based on `FreeSpaceConfig` and `TerminalPlannerConfig` update.
- **Location:** `/planning/planning_common/include/planning_common/types/planning_parameters.h`
- **Code:**
    - Initialization
        ```cpp
        planner::FreeSpacePlannerConfig free_space_planner;
        planner::TerminalPlannerConfig terminal_planner;
        ```
    - Load param
        ```cpp
        PlannerParameters()
        : ...
        , free_space_planner(planner::FreeSpacePlannerConfig::__getDefault__())
        , terminal_planner(planner::TerminalPlannerConfig::__getDefault__())
        , ...
        ```

- **Description:**
    - If add new configuration file, here is the place to load it into `PlannerParameters` struct.

---

### 54. `planning::BehaviorType`

- **Brief:** Modify `BehaviorTyp` struct.
- **Location:** `/planning/planning_common/include/planning_common/types/planning_types.h`
- **Code:**
    ```cpp
    enum class BehaviorType : uint8_t
    {
    INVALID = 0,
    FOLLOW_ROAD,
    GAP_FIND,
    MERGE,
    OVERTAKE,
    YIELD_TO_LANE_CHANGE,
    LANE_CHANGE,
    FREE_SPACE,
    TELEOP
    };

    /**
    * @brief Strings for printing above enums.
    */
    // !!Make sure to match these if enum is changed!!
    static const std::map<BehaviorType, std::string> behaviorTypeStrings = { { BehaviorType::INVALID, "INVALID" },
                                                                            { BehaviorType::FOLLOW_ROAD, "FOLLOW_ROAD" },
                                                                            { BehaviorType::GAP_FIND, "GAP_FIND" },
                                                                            { BehaviorType::MERGE, "MERGE" },
                                                                            { BehaviorType::OVERTAKE, "OVERTAKE" },
                                                                            { BehaviorType::YIELD_TO_LANE_CHANGE,
                                                                            "YIELD_TO_LANE_CHANGE" },
                                                                            { BehaviorType::LANE_CHANGE, "LANE_CHANGE" },
                                                                            { BehaviorType::FREE_SPACE, "FREE_SPACE" },
                                                                            { BehaviorType::TELEOP, "TELEOP" } };
    ```

- **Description:**
    - Add new `BehaviorType` here.

---

### 55. `planning::PredictedScene`

- **Brief:** Add terminal features into  `PredictedScene` class.
- **Location:** `/planning/planning_common/include/planning_common/types/predicted_scene.h`
- **Code:**
    - Include terminal mappery
        ```cpp
        #include "terminal_map_server/terminal_mappery/terminal_mappery.h"
        ```
    - Add initialization for terminal features
        ```cpp
        PredictedScene(std_msgs::Header scene_msg_header, const GCSLocation& gcs_location,
                    ...
                    int terminal_planning_state, std::string parking_spot_id,
                    std::optional<GCSLocation> terminal_unpark_handoff_pose,
                    std::unique_ptr<TerminalMappery> terminal_map);
        ```
    - Add terminal functions
        ```cpp
        /**
        * @brief Get the terminal planning state
        *
        * @return int The terminal planning state, corresponds to enum TerminalStateMachine::TerminalState
        */
        int getTerminalPlanningState() const;

        /**
        * @brief Get the terminal parking spot ID
        *
        * @return std::optional<std::string> The terminal parking spot ID
        *         This may correspond to origin/destination parking spot id based on the received terminal ID
        */
        std::optional<std::string> getTerminalParkingSpotID() const;

        /**
        * @brief Get the terminal unpark handoff point pose
        *
        * @return std::optional<GCSLocation> The terminal unpark handoff point pose, if it exists
        */
        std::optional<GCSLocation> getTerminalUnparkHandoffPose() const;

        /**
        * @brief Get the terminal map
        *
        * @return const perception_msgs::TerminalMap& The terminal map
        */
        const TerminalMappery& getTerminalMap() const;

        /**
        * @brief Check if left hand drive is enabled
        *
        * @return true if left hand drive is enabled
        */
        bool isLeftHandDriveEnabled() const;
        ```
    - Add terminal variables
        ```cpp
        int terminal_planning_state_ = 0;

        std::string parking_spot_id_;
        std::optional<GCSLocation> terminal_unpark_handoff_pose_ = std::nullopt;
        std::unique_ptr<TerminalMappery> terminal_map_;

        bool enable_left_hand_drive_ = false;
        ```

---

### 56. `planning::map_build::updateMapsAndBuildMappery`

- **Brief:** update `updateMapsAndBuildMappery` function with additional free space path input.
- **Location:** `/planning/planning_common/src/utils/map_build.cpp`
- **Code:**
    ```cpp
    std::unique_ptr<const Mappery> updateMapsAndBuildMappery(..., const std::vector<Eigen::Vector3d>* free_space_path)
    ```

---

### 57. `reference_line_utils.h`

- **Brief:** Utility functions for generating reference lines used in planning. This class provides methods to generate reference lines from different sources, including lane IDs, polylines, and splines.
- **Location:** `/planning/planning_common/include/planning_common/utils/reference_line_utils.h`
- **Code:**
    ```cpp
    class ReferenceLineUtils
    {
    public:
        /**
         * @brief Generate reference lines from lane IDs.
         * @param params Planner parameters.
         * @param frenet_graph Frenet representation of the map.
         * @param ego_object Ego vehicle representation.
         * @param lane_graph Lane structure in the map.
         * @param lane_id_tracker Tracks lane IDs for reference line generation.
         * @param is_teleop_active Whether teleoperation mode is active.
         * @param limit_reference_lines If true, limits the number of reference lines generated.
         * @return ReferenceLines object or std::nullopt if generation failed.
         */
        static std::optional<ReferenceLines> generateReferenceLines(const PlannerParameters& params,
                                                                    const FrenetGraph& frenet_graph,
                                                                    const EgoObject& ego_object,
                                                                    const Mappery& lane_graph,
                                                                    const LaneIDTracker& lane_id_tracker,
                                                                    bool is_teleop_active,
                                                                    bool limit_reference_lines);

        /**
         * @brief Generate a reference line from a coarsely sampled polyline.
         * @param path Coarsely sampled path points.
         * @param new_id ID of the reference line to be generated.
         * @param dS Sampling discretization.
         * @param rear_connect_point Optional connection point at the rear.
         * @return A reference line data structure, or std::nullopt if generation failed.
         */
        static std::optional<ReferenceLine> generatePolylineReferenceLine(const std::vector<CurvePoint>& path,
                                                                          LaneID new_id,
                                                                          double dS,
                                                                          const ReferencePoint* rear_connect_point = nullptr);

        /**
         * @brief Generate a lane path geometry from a Bspline.
         * @param lane_id Lane ID for the reference line.
         * @param map Map representation.
         * @param id_tracker Lane ID tracker.
         * @return Vector of geometry points representing the lane path.
         */
        static std::vector<geometry_msgs::Point> generateGeometryLanePath(LaneID lane_id,
                                                                          const maps::LaneSubMap& map,
                                                                          const LaneIDTracker& id_tracker);

        /**
         * @brief Convert lane path geometry into curved points.
         * @param lane_path_geometry Vector of geometry points representing the lane path.
         * @param params Planner parameters.
         * @return Vector of curve points.
         */
        static std::vector<CurvePoint> generateCurvedPointLanePath(
            const std::vector<geometry_msgs::Point>& lane_path_geometry,
            const PlannerParameters& params);

        /**
         * @brief Generate a spline-based reference line.
         * @param lane_path_points Points representing the lane path.
         * @param ego_heading Ego vehicle heading.
         * @param lane_id Lane ID for the reference line.
         * @param is_teleop_active Whether teleoperation mode is active.
         * @return A spline-based reference line, or std::nullopt if generation failed.
         */
        static std::optional<ReferenceLine> buildSplineReferenceLine(const Eigen::Matrix2Xd& lane_path_points,
                                                                     const Eigen::Vector2d& ego_heading,
                                                                     LaneID lane_id,
                                                                     const bool is_teleop_active);

        /**
         * @brief Build a spline-based reference line from a lane ID.
         * @param init_lane_id Lane ID.
         * @param params Planner parameters.
         * @param frenet_graph Frenet representation.
         * @param map Map representation.
         * @param id_tracker Lane ID tracker.
         * @param is_teleop_active Whether teleoperation mode is active.
         * @return A spline-based reference line, or std::nullopt if generation failed.
         */
        static std::optional<ReferenceLine> buildSplineReferenceLineFromLaneID(
            LaneID init_lane_id, const PlannerParameters& params, const FrenetGraph& frenet_graph,
            const maps::LaneSubMap& map, const LaneIDTracker& id_tracker, bool is_teleop_active);

        /**
         * @brief Build a polyline-based reference line from a lane ID.
         * @param init_lane_id Lane ID.
         * @param params Planner parameters.
         * @param frenet_graph Frenet representation.
         * @param map Map representation.
         * @param connect_point Optional connection point.
         * @param id_tracker Lane ID tracker.
         * @return A polyline reference line, or std::nullopt if generation failed.
         */
        static std::optional<ReferenceLine> buildPolylineReferenceLineFromLaneID(
            LaneID init_lane_id, const PlannerParameters& params, const FrenetGraph& frenet_graph,
            const maps::LaneSubMap& map, const ReferencePoint* connect_point, const LaneIDTracker& id_tracker);
    };
    ```
- **Description:**  
    - Provides utility functions for generating reference lines used in motion planning.  
    - Supports reference line generation from lane IDs, polylines, and Bspline representations.  
    - Implements methods for converting lane path geometry into structured reference lines.  
    - Allows fine-tuned control over reference line generation with optional constraints.

---

### 58. `planning::LaneID`

- **Brief:** update enum class `LaneID`.
- **Location:** `/planning/lane_id_tracker/include/lane_id_tracker/lane_id_common.h`
- **Code:**
    ```cpp
    enum class LaneID : std::int32_t
    {
    UNDEFINED = -1,

    // Shoulders do not have a proper super lane id.
    // To still be able to keep track of all shoulder
    // objects, the shoulder ids are hard coded to this values
    LEFT_SHOULDER = -2,
    RIGHT_SHOULDER = -3,
    PARKING_LANE = -4
    };
    ```

---

### 59. `/planner/ms_planner_node.h`

- **Brief:** MS Planner Node. This class defines a ROS node for managing motion planning using stateful tracking, localization input, and trajectory generation.
- **Location:** `/planning/planner/include/planner/ms_planner_node.h`
- **Code:**
    ```cpp
    class MSPlannerNode : public stateful::Stateful
    {
    public:
        explicit MSPlannerNode(ros::NodeHandle* nh);

    protected:
        void clear_state() override {}

    private:
        ros::NodeHandle* nh_;

        ros::Subscriber localization_sub_;
        diagnostics_utils::PublisherWrapper<planning_msgs::PlannerOutTrajectory> trajectory_pub_;
        ros::Subscriber trigger_sub_;

        CartesianState latest_received_odom_pose_;
        Eigen::Vector3d latest_received_localization_utm_;
        Eigen::Vector3d route_starting_point_;

        std::vector<Eigen::Vector2d> route_points_;
        void InitializeRoutePointsWithStartingPoint();
        void InitializeRoutePointsFromTxt();

        Eigen::Vector3d GCSToUTM(double longitude, double latitude, double heading);

        void LocalizationCallback(const perception_msgs::Localization& msg);
        void TriggerEstimationCallback(const perception_msgs::TriggerEstimation& msg);
        std::vector<Eigen::Vector3d> GenerateTrajectory();
        void PublishTrajectory();

        double trajectory_static_velocity_;
        bool initialize_route_from_txt_;
        std::string route_gcs_file_;
    };
    ```
- **Description:**  
    - Implements a motion planning node using ROS, managing localization, trajectory generation, and route handling.  
    - Listens to localization (`perception_msgs::Localization`) and trigger estimation (`perception_msgs::TriggerEstimation`) messages.  
    - Maintains route points initialized from a starting position or a `.txt` file containing longitude, latitude, and heading.  
    - Converts GCS coordinates to UTM for planning calculations.  
    - Publishes planned trajectories using `planning_msgs::PlannerOutTrajectory`.  

---

### 60. `planning::Planner`

- **Brief:** Add terminal planner feature into the `Planner` class.
- **Location:**
    - `/planning/planner/include/planner/planner.h`
    - `/planning/planner/src/planner.cpp`
- **Code:**
    - Additional functions
        ```cpp
        bool isFreeSpacePlanReady() const;

        std::optional<std::vector<Eigen::Vector3d>> getFreeSpacePath() const;
        ```
    - Terminal planner variable. From `behavior_planner/terminal_planner.h`
        ```cpp
        TerminalPlanner terminal_planner_;
        ```
    - Set free space plan flag as false
        ```cpp
        bool free_space_plan_ready_ = false;
        ```
    - Modify `runPlanningCycle` function for terminal feature
        ```cpp
        bool Planner::runPlanningCycle(const std::shared_ptr<const PredictedScene>& scene, const SourceFrame& source_frame,
                               const SituationConstraints& situation_constraints,
                               const OperationalDomain& operational_domain, PlannerDebugInfo* debug_info,
                               const AutoEngageStatus& engage_status)
        {
            ...
            } else if (terminal_sm::TerminalStateMachine::IsTerminalPlanningActive(scene->getTerminalPlanningState())) {
                action_target_per_behavior =
                    terminal_planner_.planActionTargets(situation_constraints, scene, operational_domain,
                                                        &debug_info->behavior_planner_debug_info, free_space_plan_ready_);
            } else {
                action_target_per_behavior = behavior_planner_.planActionTargets(situation_constraints, scene, operational_domain,
                                                                                &debug_info->behavior_planner_debug_info);
            }
            ...
            if (!terminal_sm::TerminalStateMachine::IsTerminalPlanningActive(scene->getTerminalPlanningState())) {
                free_space_plan_ready_ = false; // Reset free space plan ready flag when not in terminal planning
            }
        }
        ```

---

### 61. `planning::PlannerNode`

- **Brief:** Add terminal state machine into the `PlannerNode` class.
- **Location:**
    - `/planning/planner/include/planner/planner_node.h`
    - `/planning/planner/nodes/planner/planner_node.cpp`
- **Code:**
    - Additional member functions
        ```cpp
        /**
        * @brief Updates terminal state machine with current scene
        *
        * @param[in] scene_msg   the current scene
        */
        void evaluateTerminalState(const perception_msgs::Scene& scene_msg);


        /**
        * @brief Initialize terminal state machine with route waypoints
        * Also populates terminal_waypoints_ and route_polyline_ to simplify ego travel computation
        *
        * @param[in] scene_msg   the current scene
        *
        */
        void initializeTerminalStateMachine(const perception_msgs::Scene& scene_msg);

        /**
        * Compute ego travel on route based on ego state
        *
        * @param ego_state: ego state message
        * @return: ego travel on polyline consisting of only terminal waypoints (Parking Spot, Inflow, Outflow)
        */
        double computeEgoTravelOnRoute(const perception_msgs::EgoState& ego_state) const;
        ```
    - Terminal state machine variable declaration
        ```cpp
        // State machine for determining whether to use terminal planner
        std::unique_ptr<terminal_sm::TerminalStateMachine> terminal_state_machine_ = nullptr;

        // State machine for stopping and going at terminal stop lines
        std::unique_ptr<terminal_stop_go_sm::TerminalStopGoStateMachine> terminal_stop_go_state_machine_ = nullptr;

        std::vector<Eigen::Vector3d> route_polyline_; // Polyline of terminal state machine waypoints in UTM
        std::vector<double> route_travel_;            // Travel distance along route_polyline_
        ```
    - Unknown helper function, which are defined out side of `PlannerNode` class
        ```cpp
        std::optional<terminal_sm::WaypointType> getTerminalPlanningWaypointType(size_t wp_idx, size_t num_wp, uint8_t wp_type)
        {
        if (wp_idx == 0) {
            CHECK(wp_type == perception_msgs::MapWaypoint::WAYPOINT_TRIP_ORIGIN ||
                wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_SPOT);
            return terminal_sm::WaypointType::START;
        }
        if (wp_idx == num_wp - 1) {
            CHECK(wp_type == perception_msgs::MapWaypoint::WAYPOINT_TRIP_DESTINATION ||
                wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_SPOT);
            return terminal_sm::WaypointType::END;
        }
        if (wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_INFLOW) {
            return terminal_sm::WaypointType::PARKING_HANDOFF;
        }
        if (wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_OUTFLOW) {
            return terminal_sm::WaypointType::UNPARKING_HANDOFF;
        }
        return std::nullopt;
        }

        std::optional<terminal_stop_go_sm::WaypointType> getTerminalStopGoWaypointType(size_t wp_idx, size_t num_wp,
                                                                                    uint8_t wp_type)
        {
        if (wp_idx == 0) {
            CHECK(wp_type == perception_msgs::MapWaypoint::WAYPOINT_TRIP_ORIGIN ||
                wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_SPOT);
            return terminal_stop_go_sm::WaypointType::START;
        }
        if (wp_idx == num_wp - 1) {
            CHECK(wp_type == perception_msgs::MapWaypoint::WAYPOINT_TRIP_DESTINATION ||
                wp_type == perception_msgs::MapWaypoint::WAYPOINT_PARKING_SPOT);
            return terminal_stop_go_sm::WaypointType::END;
        }
        if (wp_type == perception_msgs::MapWaypoint::WAYPOINT_STOP_LINE) {
            return terminal_stop_go_sm::WaypointType::STOP_LINE;
        }
        return std::nullopt;
        }

        // Returns a straight line path in UTM coordinates starting from ego state and going straight ahead
        std::vector<Eigen::Vector3d> createStraightLinePath(const perception_msgs::EgoState ego_state)
        {
        std::vector<Eigen::Vector3d> path;
        Eigen::Vector3d ego_utm;
        const map_utils::UtmZone zone = map_utils::getUtmZone(ego_state.latitude, ego_state.longitude);
        map_utils::convertGpsToUtm(ego_state.latitude, ego_state.longitude, zone, &ego_utm[0], &ego_utm[1]);
        const double ego_utm_theta = map_utils::getUtmTheta(zone, ego_state.heading, ego_state.latitude, ego_state.longitude);
        ego_utm[2] = ego_utm_theta;
        const Eigen::Vector3d offset = Eigen::Vector3d(std::cos(ego_utm_theta), std::sin(ego_utm_theta), 0.0);
        for (size_t i = 0; i < 10; i++) {
            path.push_back(ego_utm + i * offset);
        }
        return path;
        }
        ```
    - Run terminal state machine in `runPlanner` function
        ```cpp
        void PlannerNode::runPlanner(const perception_msgs::Scene& scene_msg,
                             diagnostics_utils::DiagnosticsPublisher& node_diagnostics_publisher)
        {
            ...
            // Update terminal state machine
            evaluateTerminalState(scene_msg);
            ...
            std::optional<std::vector<Eigen::Vector3d>> free_space_path = planner_->getFreeSpacePath();
            if (!free_space_path &&
                terminal_state_machine_->CurrentStateID() == terminal_sm::TerminalStateMachine::StateIndex::UNPARK) {
            // If free space path not ready but in UNPARK, use straight line path for ego lane association
            // TODO(xinda): add an unconnected lane per parking spot for correct ego lane association
            free_space_path = createStraightLinePath(scene_msg.ego_state);
            }
            const std::vector<Eigen::Vector3d>* free_space_path_ptr =
                free_space_path.has_value() ? &free_space_path.value() : nullptr;
            ...
            std::unique_ptr<const Mappery> mappery = map_build::updateMapsAndBuildMappery(
                scene_msg.map, scene_msg.route, mission_, last_known_ego_lane_ref_, lane_id_tracker_, free_space_path_ptr);
            ...

            // Generate scenery
            exec_timing.start("buildPredictedScene", CALLER_INFO());
            const std::shared_ptr<const PredictedScene> scene = scene_builder_->buildPredictedScene(
                scene_msg, std::move(mappery), lane_id_tracker_, speed_limit_tracker_, *terminal_state_machine_,
                *terminal_stop_go_state_machine_, scene_building_options_);
        }
        ```
- **Description**: `terminal_state_machine_` is updated in `evaluateTerminalState` function and the `terminal_planner_` is called in `runPlanningCycle` function in `Planner` class.

---

### 62. `planning::PredictedSceneBuilder`

- **Brief:** Modify `buildPredictedScene` function by adding two more input parameters: `terminal_state_machine` and `terminal_stop_go_state_machine`.
- **Location:**
    - `/planning/predicted_scene_builder/include/predicted_scene_builder/predicted_scene_builder.h`
- **Code:**
    ```cpp
    /**
    * @brief Build the predicted scene
    * @param scene_msg                      Perception scene (objects, ego state, etc.)
    * @param map                            Mappery (lane graph & geometry)
    * @param lane_id_tracker                Object responsible for keeping lane IDs consistent frame-to-frame
    * @param terminal_state_machine         State machine that determines if terminal planning should be active
    * @param terminal_stop_go_state_machine State machine that determines if ego should stop or go at terminal stop lines
    * @param options                        Config for building the scene
    * @return                               A fully constructed PredictedScene
    */
    std::unique_ptr<planning::PredictedScene>
    buildPredictedScene(const perception_msgs::Scene& scene_msg, std::unique_ptr<const Mappery>&& map,
                        const LaneIDTracker& lane_id_tracker, SpeedLimitTracker& speed_limit_tracker,
                        const terminal_sm::TerminalStateMachine& terminal_state_machine,
                        const terminal_stop_go_sm::TerminalStopGoStateMachine& terminal_stop_go_state_machine,
                        SceneOptions options = SceneOptions());
    ```
- **Description**: Add `terminal_state_machine` into the scene here.

---

### 63. `PlannerRunBuildScene::buildScene`

- **Brief:** Update the `buildScene` function to instantiate `SpeedLimitTracker`, `terminal_state_machine`, and `terminal_stop_go_state_machine` before passing them as arguments to `buildPredictedScene`.
- **Location:**
    - `/planning/predicted_scene_builder/src/planner_run_build_scene.cpp`
- **Code:**
    ```cpp
    void PlannerRunBuildScene::buildScene()
    {
      scene_builder_ = std::make_shared<PredictedSceneBuilder>();
      SpeedLimitTracker speed_limit_tracker;
      auto terminal_state_machine =
          terminal_sm::TerminalStateMachine(std::vector<terminal_sm::Waypoint>{}, terminal_sm::TerminalState{});
      auto terminal_stop_go_state_machine = terminal_stop_go_sm::TerminalStopGoStateMachine(
          std::vector<terminal_stop_go_sm::Waypoint>{}, terminal_stop_go_sm::TerminalState{});
      scene_ = scene_builder_->buildPredictedScene(scene_msg_, std::move(mappery_), lane_id_tracker_, speed_limit_tracker,
                                                   terminal_state_machine, terminal_stop_go_state_machine);
    }
    ```
- **Description:** This change introduces the creation of instances for `SpeedLimitTracker`, `terminal_state_machine`, and `terminal_stop_go_state_machine`, which are then passed to the updated `buildPredictedScene` function.

---

### 64. `PredictedSceneBuilder` - `predicted_scene_builder.cpp`

- **Brief:** Multiple changes related to adding new objects, constants, and modifying functions, such as updating the `buildPredictedScene` function to support additional parameters, and adding logic for stop lines and terminal maps.
- **Location:**
    - `/planning/predicted_scene_builder/src/predicted_scene_builder.cpp`
  
- **Code Block 1**: New includes and constants
    ```cpp
    #include <geometry_msgs/Point32.h>
    #include "planning_common/utils/reference_line_utils.h"
    #include "terminal_map_server/terminal_mappery/terminal_mappery.h"

    // Threshold for the time (seconds) it takes for ego to reach next stop line
    constexpr double STOP_LINE_TIME_THRESHOLD = 6.0;
    // Threshold for the distance (meters) to the next stop line
    constexpr double STOP_LINE_DISTANCE_THRESHOLD = 20.0;
    ```
- **Code Block 2**: New function `getStaticObjectBoundaries`
    ```cpp
    using StaticObjectBoundaries = std::unordered_map<ObjectID, std::pair<Eigen::Vector2d, perception_msgs::HullInfo>>;
    StaticObjectBoundaries getStaticObjectBoundaries(const std::unordered_map<ObjectID, PredictedObject>& predicted_objects)
    {
    StaticObjectBoundaries static_obstacle_boundaries;
    for (const auto& [obj_id, pred_obj] : predicted_objects) {
        if (pred_obj.isObjectStatic()) {
        Eigen::Vector2d centerpoint(pred_obj.getCartesianState().x, pred_obj.getCartesianState().y);
        static_obstacle_boundaries[obj_id] = std::make_pair(centerpoint, pred_obj.getConvexHull());
        }
    }
    return static_obstacle_boundaries;
    }
    ```
- **Code Block 3**: New function `convertStopLineToSceneObject`
    ```cpp
    perception_msgs::Object convertStopLineToSceneObject(const perception_msgs::MapWaypoint& waypoint,
                                                        const perception_msgs::EgoState& ego_state)
    {
    // Code for converting stop line waypoint to a scene object
    }
    ```
- **Code Block 4**: Modifying `buildPredictedScene` function
    ```cpp
        std::unique_ptr<planning::PredictedScene> PredictedSceneBuilder::buildPredictedScene(
        const perception_msgs::Scene& scene_msg, std::unique_ptr<const Mappery>&& map, const LaneIDTracker& lane_id_tracker,
        SpeedLimitTracker& speed_limit_tracker, const terminal_sm::TerminalStateMachine& terminal_state_machine,
        const terminal_stop_go_sm::TerminalStopGoStateMachine& terminal_stop_go_state_machine, SceneOptions options)
    {
    // Modifications to buildPredictedScene to incorporate speed limit tracking, terminal state machine, and stop line objects
    }

    ```

- **Code Block 5**: Updates for predicted objects
    ```cpp
    // Check for any Construction or Debris Objects in Scene
    bool out_of_odd_obj_detected = false;
    for (const perception_msgs::Object& object : scene_msg.objects) {
        PredictedObject predicted_obj(object, ego_object.getCartesianState(), ego_object.getYawRate());
        // Additional checks for debris objects
        predicted_objects.emplace(id, std::move(predicted_obj));
        objects_observed_by_camera.emplace(id, object.was_observed_by_camera);
        }
        ```
    - **Code Block 6**: Changes for stop line processing
        ```cpp
        for (const perception_msgs::MapWaypoint& waypoint : scene_msg.route.waypoints) {
    if (waypoint.waypoint_type == perception_msgs::MapWaypoint::WAYPOINT_STOP_LINE &&
        waypoint.id == terminal_stop_go_state_machine.CurrentWaypointId()) {
        // Logic for adding stop line object to predicted objects
    }
    }
    ```
- **Code Block 6**: Changes for stop line processing
    ```cpp
    for (const perception_msgs::MapWaypoint& waypoint : scene_msg.route.waypoints) {
    if (waypoint.waypoint_type == perception_msgs::MapWaypoint::WAYPOINT_STOP_LINE &&
        waypoint.id == terminal_stop_go_state_machine.CurrentWaypointId()) {
        // Logic for adding stop line object to predicted objects
    }
    }
    ```
- **Code Block 7**: Creating and using the terminal map
    ```cpp
    // Create Terminal Map
    std::unique_ptr<TerminalMappery> terminal_map = std::make_unique<TerminalMappery>(scene_msg.terminal_map);
    terminal_map->loadStaticObjectsInTerminal(getStaticObjectBoundaries(predicted_objects));
    ```

- **Code Block 8**: Handling parking spot and unpark handoff pose
    ```cpp
    // Get parking spots for origin and destination, if they exist in current terminal
    std::string parking_spot_id = "";
    if (scene_msg.terminal_map.terminal_id == scene_msg.route.origin_parking_spot.terminal_id) {
    parking_spot_id = scene_msg.route.origin_parking_spot.id;
    }
    // Get terminal unpark handoff point pose
    std::optional<GCSLocation> terminal_unpark_handoff_pose = std::nullopt;
    ```
- **Code Block 9**: Adjustments to `buildPredictedScene` return statement
    ```cpp
    auto new_predicted_scene = std::make_unique<PredictedScene>(
        scene_msg.header, ego_location, std::move(map), std::move(ego_object), std::move(predicted_objects),
        std::move(objects_per_lane), std::move(relevant_vos), std::move(ignored_vos), out_of_odd_obj_detected,
        std::move(frenet_graph), std::move(*reference_lines), vehicle_odom_T, motion_history, lane_change_request,
        desired_speed_limit, std::move(cache), std::move(lane_id_tracker), scene_msg.teleop_output,
        scene_msg.in_teleop_zone, scene_msg.manual_lane_change_enabled.data, std::move(distances_to_route_end),
        options.desired_speed_above_road_limit, terminal_planning_state, parking_spot_id, terminal_unpark_handoff_pose,
        std::move(terminal_map));
    ```
- **Code Block 10**: Fix for `getEgoLane`
    ```cpp
    if (lane_graph.getEgoLane() != nullptr) {
    const mappery::SuperLane& ego_lane = lane_graph.getEgoLane()->ego_lane;
    auto verified_ego_lane_id = lane_id_tracker.verifyAndGetLaneID(ego_lane.lane_refs);
    assert(verified_ego_lane_id);
    lane_id_association = *verified_ego_lane_id;
    }
    ```
- **Code Block 11**: Changes for `getNextMergeJunction`
    ```cpp
    boost::optional<lane_map::JunctionRef> next_merging_junction_in_route = boost::none;
    if (lane_graph.getEgoLaneRef()) {
        next_merging_junction_in_route = lane_map_utils::getNextMergeJunction(
            *lane_graph.getEgoLaneRef(), lane_graph.getMap(),
            lane_map_utils::forwardRouteLaneFollower(lane_graph.getMap(), lane_graph.getRouteLaneGroups()));
    }
    ```

- **Description**:  Multiple updates were made to add new functionality for handling stop lines, static object boundaries, and terminal maps. Additionally, the buildPredictedScene method now supports new parameters for handling speed limits, terminal state machines, and stop lines. Several constants related to thresholds for stop line processing were also introduced.

---

### 65. `planning::fillCostsOuterToInner` and `planning::fillCostsInnerToOuter`

- **Brief:** Refactor functions for lane preference cost calculations by renaming and altering the directionality of lane cost computations.
- **Location:**
    - `/planning/situation_awareness/include/situation_awareness/static_costs/lane_cost_builders/lane_preference_cost_builder_helpers/lane_preferences_cost_builder_helpers.h`

- **Description**: change name from `fillCostsRightToLeft` to `fillCostsOuterToInner`, `fillCostsLeftToRight` to `fillCostsInnerToOuter`.

---

### 66. `planning::findMergingLaneRefs`

- **Brief:** Rename function to generalize its purpose for finding merging lane references.
- **Location:**
    - `/planning/situation_awareness/include/situation_awareness/static_costs/lane_cost_builders/merging/merging_utils.h`

- **Description**: change name from `rightMergingLaneRefs` to `findMergingLaneRefs`.

---

### 67. `planning::outerMostLaneCost`

- **Brief:** Rename function to generalize its purpose for the outermost lane cost, not just the rightmost lane.
- **Location:**
    - `/planning/situation_awareness/include/situation_awareness/static_costs/lane_cost_builders/vehicle_on_shoulder_cost_builder.h`

- **Description**: change name from `rightMostLaneCost` to `outerMostLaneCost`.

---

### 68. `planning::SituationSpeedLimiter::updateCurrentSpeedLimit`

- **Brief:** Add a check for valid `EgoLaneRef` before retrieving speed limits from the limiters.
- **Location:**
    - `/planning/situation_awareness/src/speed_limiter/situation_speed_limiter.cpp`

- **Code**: Adding validation for `EgoLaneRef`
    ```cpp
    if (!scene.getLaneGraph().getEgoLaneRef()) {
    suggested_speed_from_limiters[type] = {};
    } else {
    suggested_speed_from_limiters[type] = limiter->getLimits(scene);
    }
    ```
---

### 69. `planning::LaneCostBuilder::LaneCostBuilder`

- **Brief:** Add conditional activation for `EndOfRouteCostBuilder` and introduce a check for `EgoLaneRef` before adding cost policies.
- **Location:**
    - `/planning/situation_awareness/src/static_costs/lane_cost_builder.cpp`

- **Code**: Validate EgoLaneRef before adding policies
    ```cpp
    if (!scene.getLaneGraph().getEgoLaneRef()) {
        combined_policy.addPolicy(cost_type, LaneCostPolicy());
    } else {
        const LaneCostPolicy& policy = builder->buildCostFunction(scene);
        combined_policy.addPolicy(cost_type, policy);
    }
    ```
- **Description**: A check is added to ensure that `EgoLaneRef` is valid before computing lane cost policies. If `EgoLaneRef` is null, an empty `LaneCostPolicy` is added to combined_policy, preventing invalid accesses and ensuring stability.

---

### 70. `SpeedLimitTracker` Class

- **Brief:** Introduce a new `SpeedLimitTracker` class to track and determine the desired speed limit based on scene data, ego state, and terminal conditions.
- **Location:**
    - `/planning/speed_limit_tracker/include/speed_limit_tracker/speed_limit_tracker.h`

- **Code**: `SpeedLimitTracker` Class Definition

    ```cpp
    class SpeedLimitTracker
    {
    public:
    SpeedLimitTracker() : last_dynamic_speed_limit_(100.0)
    {
    }

    const PredictedScene::DesiredSpeedLimit
    getDesiredSpeedLimit(const perception_msgs::Scene& scene_msg, const Mappery& map,
                        const std::unordered_map<ObjectID, PredictedObject>& predicted_objects, const EgoObject& ego,
                        const terminal_sm::TerminalStateMachine& terminal_state_machine,
                        const double desired_speed_above_road_limit, const double terminal_parking_speed_limit);

    private:
    double last_dynamic_speed_limit_;

    std::optional<double> updateSpeedLimitSign(const std::unordered_map<ObjectID, PredictedObject>& predicted_objects,
                                                const EgoObject& ego, const Mappery& map);
    };
    ```
- **Description:**
  - **Introduces the `SpeedLimitTracker` class**, responsible for tracking and determining the desired speed limit based on various factors.
  - **Maintains the last dynamic speed limit** using `last_dynamic_speed_limit_`, initialized to `100.0`.
  - **Provides the function `getDesiredSpeedLimit()`**, which calculates the speed limit considering:
    - **Scene data (`scene_msg`)** - includes perception information.
    - **Road mapping (`map`)** - retrieves lane and road information.
    - **Predicted objects (`predicted_objects`)** - factors in detected obstacles and vehicles.
    - **Ego vehicle state (`ego`)** - considers current vehicle dynamics.
    - **Terminal state machine (`terminal_state_machine`)** - applies rules for speed adjustment in terminal areas.
    - **Speed constraints** - incorporates limits from `desired_speed_above_road_limit` and `terminal_parking_speed_limit`.
  - **Defines a private helper function `updateSpeedLimitSign()`**, which processes detected speed limit signs and updates the speed limit accordingly.

--- 

### 71. `TerminalMappery` Class

- **Brief:** Introduce the `TerminalMappery` class to construct a terminal environment representation in the ego vehicle's frame using data from `perception_msgs::TerminalMap`. The class provides methods for retrieving static obstacles and parking spot boundaries within a given distance.
- **Location:**
    - `/planning/terminal_map_server/include/terminal_map_server/terminal_mappery/terminal_mappery.h`

- **Code**
    ```cpp
    class TerminalMappery
    {
    public:
    /**
    * @brief Construct a new TerminalMappery object
    *
    * @param terminal_map The terminal map
    */
    TerminalMappery(const perception_msgs::TerminalMap& terminal_map);

    /**
    * @brief Construct a new (empty) TerminalMappery object
    */
    TerminalMappery();

    /**
    * @brief Load static objects in terminal
    *
    * @param predicted_objects The predicted objects (ObjectID, centerpoint, hull info)
    */
    void loadStaticObjectsInTerminal(
        const std::unordered_map<ObjectID, std::pair<Eigen::Vector2d, perception_msgs::HullInfo>>& predicted_objects);

    /**
    * @brief Get all static boundaries of Objects in terminal within a certain distance
    *
    * @param distance_m The distance in meters
    */
    std::vector<std::vector<Eigen::Vector2d>> getStaticObstaclesBoundariesWithinDistance(double distance_m) const;

    /**
    * @brief Get the terminal parking boundaries for a given terminal in Vehicle Frame within a certain distance
    *
    * @param distance_m The distance in meters
    * @return std::vector<std::vector<Eigen::Vector2d>> The terminal parking boundaries in Vehicle Frame (no heading)
    */
    std::vector<std::vector<Eigen::Vector2d>> getTerminalParkingBoundariesWithinDistance(double distance_m) const;

    /**
    * @brief Get terminal parking spot for the given parking spot ID
    *
    * @param parking_spot_id The parking spot ID
    * @return std::optional<lane_map::TerminalParkingSpot> The terminal parking spot, if it exists
    */
    std::optional<lane_map::TerminalParkingSpot> getTerminalParkingSpot(const std::string& parking_spot_id) const;

    /**
    * @brief Get the terminal map
    *
    * @return const perception_msgs::TerminalMap& The terminal map
    */
    const lane_map::TerminalMap& getTerminalMap() const;

    private:
    struct StaticObstacleBoundary
    {
        Eigen::Vector2d centerpoint;
        std::vector<Eigen::Vector2d> boundary;
    };

    // Terminal Map
    lane_map::TerminalMap terminal_map_;

    // Map Frame
    maps::MapFrame map_frame_;

    // Static Object Boundaries / Parking Spots in Terminal
    std::vector<std::vector<Eigen::Vector2d>> static_obstacles_boundaries_;
    std::vector<lane_map::TerminalParkingSpot> terminal_parking_spots_;

    // KDTree for static obstacles and parking spots
    std::unique_ptr<utils_association::PointToPointPairer> static_obstacles_kdtree_;
    std::unique_ptr<utils_association::PointToPointPairer> parking_spots_kdtree_;

    /**
    * @brief Load parking spots in terminal
    */
    void loadParkingSpotsInTerminal();

    /**
    * @brief Convert a polygon from UTM to Vehicle Frame
    *
    * @param polygon The polygon in UTM
    * @return std::vector<Eigen::Vector2d> The polygon in Vehicle Frame (no heading)
    */
    std::vector<Eigen::Vector2d> convertPolygonUtmToVehicleFrame(const std::vector<Eigen::Vector2d>& polygon) const;

    /**
    * @brief Convert a point from UTM to Vehicle Frame
    *
    * @param utm_point The point in UTM
    * @return Eigen::Vector2d The point in Vehicle Frame (no heading)
    */
    Eigen::Vector2d convertPointUtmToVehicleFrame(const Eigen::Vector2d& utm_point) const;

    /**
    * @brief Convert a point from Vehicle Frame to UTM
    *
    * @param vehicle_point The point in Vehicle Frame
    * @return Eigen::Vector2d The point in UTM
    */
    Eigen::Vector2d convertPointVehicleFrameToUtm(const Eigen::Vector2d& vehicle_point) const;
    };

    ```

- **Description**: `TerminalMappery` class is a direct representation of `perception_msgs::TerminalMap`. It is generated in `PredictedScene` and used for `planFreeSpacePath` and `rasterizeCostMap`. 
