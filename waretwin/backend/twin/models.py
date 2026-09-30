from django.db import models
from django.utils import timezone

class EventLog(models.Model):
    run_id = models.CharField(max_length=32, db_index=True)
    event_id = models.CharField(max_length=64)
    tick = models.IntegerField(default=0, db_index=True)
    type = models.CharField(max_length=64, db_index=True)
    source = models.CharField(max_length=32, blank=True)
    severity = models.CharField(max_length=16, db_index=True)
    message = models.TextField(blank=True)
    robot_id = models.CharField(max_length=32, blank=True, null=True, db_index=True)
    task_id = models.CharField(max_length=32, blank=True, null=True)
    zone_id = models.CharField(max_length=32, blank=True, null=True, db_index=True)
    conveyor_id = models.CharField(max_length=32, blank=True, null=True)
    camera_id = models.CharField(max_length=32, blank=True, null=True)
    payload = models.JSONField(default=dict, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-id']
        indexes = [models.Index(fields=['run_id', '-tick'])]

    def to_twin_dict(self):
        data = {
            'id': self.event_id,
            'tick': self.tick,
            'type': self.type,
            'source': self.source,
            'severity': self.severity,
            'message': self.message,
        }
        for key in ('robot_id', 'task_id', 'zone_id', 'conveyor_id', 'camera_id'):
            value = getattr(self, key)
            if value:
                data[key] = value
        if self.payload:
            data['payload'] = self.payload
        return data

class RobotEndpoint(models.Model):
    """Persistent endpoint metadata for hardware/bridge fleet integrations."""
    robot_id = models.CharField(max_length=32, unique=True)
    name = models.CharField(max_length=128, blank=True)
    base_url = models.URLField(blank=True)
    ws_url = models.URLField(blank=True)
    enabled = models.BooleanField(default=True)
    last_seen = models.DateTimeField(null=True, blank=True)
    metadata = models.JSONField(default=dict, blank=True)


class RobotVda5050Configuration(models.Model):
    """Robot-scoped VDA5050 broker and runtime settings.

    The MQTT password is an encrypted Fernet token. It is deliberately excluded
    from all serializers and diagnostics; clients only receive the configured
    flag and can replace the secret without reading it back.
    """
    robot_id = models.CharField(max_length=64, unique=True, db_index=True)
    enabled = models.BooleanField(default=False)
    mqtt_host = models.CharField(max_length=255, blank=True)
    mqtt_port = models.PositiveIntegerField(default=1883)
    mqtt_username = models.CharField(max_length=128, blank=True)
    mqtt_password_ciphertext = models.TextField(blank=True)
    tls_enabled = models.BooleanField(default=False)
    topic_prefix = models.CharField(max_length=128, default='vda5050')
    interface_name = models.CharField(max_length=64, default='uagv')
    manufacturer = models.CharField(max_length=64, default='PTAGV')
    serial_number = models.CharField(max_length=64, blank=True)
    protocol_version = models.CharField(max_length=16, default='2.0.0')
    mqtt_protocol_version = models.CharField(max_length=8, default='3.1.1')
    allow_task = models.BooleanField(default=True)
    allow_instant_actions = models.BooleanField(default=True)
    auto_reconnect = models.BooleanField(default=True)
    reconnect_interval = models.PositiveIntegerField(default=5)
    connection_timeout = models.PositiveIntegerField(default=5)
    keepalive = models.PositiveIntegerField(default=30)
    client_id = models.CharField(max_length=128, blank=True)
    updated_at = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f'VDA5050<{self.robot_id}>'

class Mission(models.Model):
    STATUS = [('PENDING','PENDING'),('SENT','SENT'),('RUNNING','RUNNING'),('DONE','DONE'),('FAILED','FAILED'),('CANCELLED','CANCELLED')]
    mission_id = models.CharField(max_length=64, unique=True)
    robot_id = models.CharField(max_length=32, blank=True, null=True)
    kind = models.CharField(max_length=32, default='NAVIGATE')
    target = models.JSONField(default=dict)
    status = models.CharField(max_length=16, choices=STATUS, default='PENDING')
    command_id = models.CharField(max_length=64, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

class Warehouse(models.Model):
    STATUS = [('ACTIVE', 'ACTIVE'), ('INACTIVE', 'INACTIVE')]

    code = models.CharField(max_length=64, unique=True, db_index=True)
    name = models.CharField(max_length=160)
    description = models.TextField(blank=True)
    status = models.CharField(max_length=16, choices=STATUS, default='ACTIVE', db_index=True)
    width = models.FloatField(default=1.0)
    depth = models.FloatField(default=1.0)
    height = models.FloatField(default=1.0)
    units = models.CharField(max_length=16, default='m')
    layout_id = models.CharField(max_length=128, blank=True, db_index=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['code']

    def __str__(self):
        return f'{self.code} - {self.name}'


class Zone(models.Model):
    TYPES = [
        ('STORAGE', 'STORAGE'), ('PICKING', 'PICKING'), ('BUFFER', 'BUFFER'),
        ('CHARGING', 'CHARGING'), ('RESTRICTED', 'RESTRICTED'), ('OTHER', 'OTHER'),
    ]
    STATUS = [('ACTIVE', 'ACTIVE'), ('INACTIVE', 'INACTIVE'), ('BLOCKED', 'BLOCKED')]

    warehouse = models.ForeignKey(Warehouse, on_delete=models.PROTECT, related_name='zones')
    code = models.CharField(max_length=64)
    name = models.CharField(max_length=160)
    type = models.CharField(max_length=20, choices=TYPES, default='STORAGE', db_index=True)
    status = models.CharField(max_length=16, choices=STATUS, default='ACTIVE', db_index=True)
    floor = models.PositiveIntegerField(default=1, db_index=True)
    color = models.CharField(max_length=16, default='#3b82f6')
    polygon = models.JSONField(default=list, blank=True)
    description = models.TextField(blank=True)
    layout_zone_id = models.CharField(max_length=128, blank=True, db_index=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['warehouse_id', 'floor', 'code']
        constraints = [models.UniqueConstraint(fields=['warehouse', 'code'], name='uniq_zone_code_per_warehouse')]

    def __str__(self):
        return f'{self.warehouse.code}/{self.code}'


class Shelf(models.Model):
    TYPES = [('STORAGE', 'STORAGE'), ('PICK_FACE', 'PICK_FACE'), ('BUFFER', 'BUFFER'), ('OTHER', 'OTHER')]
    STATUS = [
        ('AVAILABLE', 'AVAILABLE'), ('OCCUPIED', 'OCCUPIED'), ('FULL', 'FULL'),
        ('RESERVED', 'RESERVED'), ('DISABLED', 'DISABLED'), ('MAINTENANCE', 'MAINTENANCE'),
    ]

    zone = models.ForeignKey(Zone, on_delete=models.PROTECT, related_name='shelves')
    code = models.CharField(max_length=64)
    name = models.CharField(max_length=160)
    type = models.CharField(max_length=20, choices=TYPES, default='STORAGE', db_index=True)
    status = models.CharField(max_length=20, choices=STATUS, default='AVAILABLE', db_index=True)
    floor = models.PositiveIntegerField(default=1, db_index=True)

    # Floor-plan coordinates: x/y are navigation-plane coordinates; z is elevation.
    position_x = models.FloatField(default=0.0)
    position_y = models.FloatField(default=0.0)
    position_z = models.FloatField(default=0.0)
    width = models.FloatField(default=1.0)
    depth = models.FloatField(default=1.0)
    height = models.FloatField(default=1.0)
    rotation_deg = models.FloatField(default=0.0)
    levels = models.PositiveIntegerField(default=1)

    capacity = models.PositiveIntegerField(default=8)
    current_load = models.PositiveIntegerField(default=0)

    # Robot navigation target beside the physical shelf.
    access_x = models.FloatField(default=0.0)
    access_y = models.FloatField(default=0.0)
    access_yaw = models.FloatField(default=0.0)

    layout_rack_id = models.CharField(max_length=128, blank=True, db_index=True)
    description = models.TextField(blank=True)
    metadata = models.JSONField(default=dict, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['zone_id', 'code']
        constraints = [models.UniqueConstraint(fields=['zone', 'code'], name='uniq_shelf_code_per_zone')]

    def __str__(self):
        return f'{self.zone}/{self.code}'


class WarehouseMap(models.Model):
    """Database-backed map for one warehouse.

    `layout` is the authoritative published geometry used by the live map/runtime.
    `draft` is the editor working copy. Every warehouse owns its own map.
    """
    warehouse = models.OneToOneField(Warehouse, on_delete=models.CASCADE, related_name='map')
    layout = models.JSONField(default=dict)
    draft = models.JSONField(default=dict)
    revision = models.PositiveIntegerField(default=1, db_index=True)
    published_version = models.PositiveIntegerField(default=0)
    is_active = models.BooleanField(default=False, db_index=True)
    updated_by = models.ForeignKey('auth.User', on_delete=models.SET_NULL, null=True, blank=True, related_name='+')
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['warehouse_id']

    def __str__(self):
        return f'Map<{self.warehouse.code}> r{self.revision}'


class WarehouseMapVersion(models.Model):
    warehouse_map = models.ForeignKey(WarehouseMap, on_delete=models.CASCADE, related_name='versions')
    version = models.PositiveIntegerField()
    revision = models.PositiveIntegerField()
    layout = models.JSONField(default=dict)
    created_by = models.ForeignKey('auth.User', on_delete=models.SET_NULL, null=True, blank=True, related_name='+')
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-version']
        constraints = [
            models.UniqueConstraint(fields=['warehouse_map', 'version'], name='uniq_map_version_per_warehouse')
        ]

    def __str__(self):
        return f'{self.warehouse_map.warehouse.code}@v{self.version}'


class WorkPoint(models.Model):
    """A robot-accessible point in a warehouse.

    Scheduler-facing abstraction for shelves, conveyors, inbound/outbound docks,
    packing/sorting stations, chargers and custom operational points. Coordinates
    are always stored in warehouse metres and are independent of screen pixels.
    """
    KINDS = [
        ('SHELF', 'SHELF'), ('CONVEYOR_IN', 'CONVEYOR_IN'), ('CONVEYOR_OUT', 'CONVEYOR_OUT'),
        ('INBOUND', 'INBOUND'), ('OUTBOUND', 'OUTBOUND'), ('PACKING', 'PACKING'),
        ('SORTING', 'SORTING'), ('BUFFER', 'BUFFER'), ('CHARGING', 'CHARGING'),
        ('DOCK', 'DOCK'), ('STATION', 'STATION'), ('PARKING', 'PARKING'), ('CUSTOM', 'CUSTOM'),
    ]

    warehouse = models.ForeignKey(Warehouse, on_delete=models.CASCADE, related_name='workpoints')
    zone = models.ForeignKey(Zone, on_delete=models.SET_NULL, null=True, blank=True, related_name='workpoints')
    code = models.CharField(max_length=96)
    name = models.CharField(max_length=180)
    kind = models.CharField(max_length=24, choices=KINDS, db_index=True)
    floor = models.PositiveIntegerField(default=1, db_index=True)
    x = models.FloatField(default=0.0)
    y = models.FloatField(default=0.0)
    z = models.FloatField(default=0.0)
    yaw = models.FloatField(default=0.0)
    resource_type = models.CharField(max_length=40, blank=True)
    resource_id = models.CharField(max_length=128, blank=True, db_index=True)
    enabled = models.BooleanField(default=True, db_index=True)
    capacity = models.PositiveIntegerField(default=1)
    metadata = models.JSONField(default=dict, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['warehouse_id', 'kind', 'code']
        constraints = [models.UniqueConstraint(fields=['warehouse', 'code'], name='uniq_workpoint_code_per_warehouse')]
        indexes = [models.Index(fields=['warehouse', 'kind', 'enabled'])]

    def __str__(self):
        return f'{self.warehouse.code}/{self.code}'


class RobotProfile(models.Model):
    """Persistent fleet metadata. Runtime telemetry remains authoritative for pose/status."""
    warehouse = models.ForeignKey(Warehouse, on_delete=models.CASCADE, related_name='robot_profiles')
    robot_id = models.CharField(max_length=64)
    name = models.CharField(max_length=160, blank=True)
    enabled = models.BooleanField(default=True, db_index=True)
    payload_capacity = models.PositiveIntegerField(default=4)
    min_dispatch_battery = models.FloatField(default=20.0)
    capabilities = models.JSONField(default=list, blank=True)
    metadata = models.JSONField(default=dict, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['warehouse_id', 'robot_id']
        constraints = [models.UniqueConstraint(fields=['warehouse', 'robot_id'], name='uniq_robot_profile_per_warehouse')]

    def __str__(self):
        return self.robot_id


class WarehouseOrder(models.Model):
    TYPES = [
        ('MOVE', 'MOVE'), ('PICK', 'PICK'), ('REPLENISH', 'REPLENISH'),
        ('INBOUND', 'INBOUND'), ('OUTBOUND', 'OUTBOUND'), ('TRANSFER', 'TRANSFER'),
    ]
    PRIORITIES = [('LOW', 'LOW'), ('NORMAL', 'NORMAL'), ('HIGH', 'HIGH'), ('CRITICAL', 'CRITICAL')]
    STATUS = [
        ('NEW', 'NEW'), ('PLANNED', 'PLANNED'), ('QUEUED', 'QUEUED'), ('RUNNING', 'RUNNING'),
        ('COMPLETED', 'COMPLETED'), ('CANCELLED', 'CANCELLED'), ('FAILED', 'FAILED'),
    ]

    warehouse = models.ForeignKey(Warehouse, on_delete=models.PROTECT, related_name='orders')
    order_no = models.CharField(max_length=64, unique=True, db_index=True)
    external_ref = models.CharField(max_length=128, blank=True, db_index=True)
    type = models.CharField(max_length=20, choices=TYPES, default='MOVE', db_index=True)
    priority = models.CharField(max_length=16, choices=PRIORITIES, default='NORMAL', db_index=True)
    status = models.CharField(max_length=16, choices=STATUS, default='NEW', db_index=True)
    source = models.ForeignKey(WorkPoint, on_delete=models.PROTECT, related_name='orders_from')
    destination = models.ForeignKey(WorkPoint, on_delete=models.PROTECT, related_name='orders_to')
    quantity = models.PositiveIntegerField(default=1)
    load_units = models.PositiveIntegerField(default=1)
    payload_weight_kg = models.FloatField(default=0.0)
    due_at = models.DateTimeField(null=True, blank=True, db_index=True)
    notes = models.TextField(blank=True)
    metadata = models.JSONField(default=dict, blank=True)
    created_by = models.ForeignKey('auth.User', on_delete=models.SET_NULL, null=True, blank=True, related_name='+')
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['-created_at']
        indexes = [models.Index(fields=['warehouse', 'status', 'priority'])]

    def __str__(self):
        return self.order_no


class ShelfInventoryItem(models.Model):
    """One physical order/load unit currently tracked inside the warehouse.

    Shelf.current_load remains the fast render counter used by the 2D/3D map,
    while this table stores the business identity of each individual item so an
    operator can click a shelf and move one exact order to another shelf or to
    outbound.
    """
    STATUS = [
        ('STORED', 'STORED'),
        ('RESERVED', 'RESERVED'),
        ('OUTBOUND', 'OUTBOUND'),
    ]

    item_uid = models.CharField(max_length=64, unique=True, db_index=True)
    warehouse = models.ForeignKey(Warehouse, on_delete=models.CASCADE, related_name='inventory_items')
    shelf = models.ForeignKey(Shelf, on_delete=models.SET_NULL, null=True, blank=True, related_name='inventory_items')
    status = models.CharField(max_length=16, choices=STATUS, default='STORED', db_index=True)
    item_code = models.CharField(max_length=128, blank=True, db_index=True)
    item_name = models.CharField(max_length=200, blank=True)
    external_ref = models.CharField(max_length=128, blank=True, db_index=True)
    quantity = models.PositiveIntegerField(default=1)
    load_units = models.PositiveIntegerField(default=1)
    payload_weight_kg = models.FloatField(default=0.0)
    origin_order = models.ForeignKey(
        WarehouseOrder, on_delete=models.SET_NULL, null=True, blank=True, related_name='originated_inventory_items'
    )
    last_movement_order = models.ForeignKey(
        WarehouseOrder, on_delete=models.SET_NULL, null=True, blank=True, related_name='last_moved_inventory_items'
    )
    reserved_by_order = models.OneToOneField(
        WarehouseOrder, on_delete=models.SET_NULL, null=True, blank=True, related_name='reserved_inventory_item'
    )
    metadata = models.JSONField(default=dict, blank=True)
    stored_at = models.DateTimeField(default=timezone.now)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['shelf_id', 'stored_at', 'id']
        indexes = [
            models.Index(fields=['warehouse', 'shelf', 'status']),
            models.Index(fields=['warehouse', 'item_code', 'status']),
        ]

    def __str__(self):
        return f'{self.item_uid} @ {self.shelf.code if self.shelf_id else self.status}'


class RobotSchedule(models.Model):
    MODES = [('AUTO', 'AUTO'), ('SEMI_AUTO', 'SEMI_AUTO'), ('MANUAL', 'MANUAL')]
    STATUS = [
        ('DRAFT', 'DRAFT'), ('PLANNED', 'PLANNED'), ('QUEUED', 'QUEUED'),
        ('RUNNING', 'RUNNING'), ('COMPLETED', 'COMPLETED'), ('FAILED', 'FAILED'),
        ('CANCELLED', 'CANCELLED'),
    ]

    schedule_id = models.CharField(max_length=64, unique=True, db_index=True)
    warehouse = models.ForeignKey(Warehouse, on_delete=models.PROTECT, related_name='robot_schedules')
    order = models.ForeignKey(WarehouseOrder, on_delete=models.PROTECT, related_name='schedules')
    robot = models.ForeignKey(RobotProfile, on_delete=models.PROTECT, related_name='schedules')
    mode = models.CharField(max_length=16, choices=MODES, default='AUTO')
    status = models.CharField(max_length=16, choices=STATUS, default='PLANNED', db_index=True)
    planned_start = models.DateTimeField(db_index=True)
    planned_end = models.DateTimeField(null=True, blank=True)
    actual_start = models.DateTimeField(null=True, blank=True)
    actual_end = models.DateTimeField(null=True, blank=True)
    estimated_distance_m = models.FloatField(default=0.0)
    estimated_duration_s = models.FloatField(default=0.0)
    score = models.JSONField(default=dict, blank=True)
    current_leg = models.PositiveIntegerField(default=0)
    engine_task_id = models.CharField(max_length=64, blank=True, db_index=True)
    failure_reason = models.TextField(blank=True)
    created_by = models.ForeignKey('auth.User', on_delete=models.SET_NULL, null=True, blank=True, related_name='+')
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['planned_start', 'schedule_id']
        indexes = [models.Index(fields=['warehouse', 'status', 'planned_start']), models.Index(fields=['robot', 'status', 'planned_start'])]

    def __str__(self):
        return self.schedule_id


class ScheduleStop(models.Model):
    ACTIONS = [('PICKUP', 'PICKUP'), ('DROPOFF', 'DROPOFF'), ('TRANSIT', 'TRANSIT'), ('WAIT', 'WAIT'), ('CHARGE', 'CHARGE')]
    STATUS = [
        ('PENDING', 'PENDING'), ('ACTIVE', 'ACTIVE'), ('ARRIVED', 'ARRIVED'),
        ('COMPLETED', 'COMPLETED'), ('SKIPPED', 'SKIPPED'), ('FAILED', 'FAILED'),
    ]

    schedule = models.ForeignKey(RobotSchedule, on_delete=models.CASCADE, related_name='stops')
    sequence = models.PositiveIntegerField()
    workpoint = models.ForeignKey(WorkPoint, on_delete=models.PROTECT, related_name='schedule_stops')
    action = models.CharField(max_length=16, choices=ACTIONS, default='TRANSIT')
    service_seconds = models.PositiveIntegerField(default=0)
    status = models.CharField(max_length=16, choices=STATUS, default='PENDING')
    arrived_at = models.DateTimeField(null=True, blank=True)
    completed_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ['schedule_id', 'sequence']
        constraints = [models.UniqueConstraint(fields=['schedule', 'sequence'], name='uniq_schedule_stop_sequence')]


class ResourceReservation(models.Model):
    STATUS = [('HELD', 'HELD'), ('ACTIVE', 'ACTIVE'), ('RELEASED', 'RELEASED'), ('CANCELLED', 'CANCELLED')]
    schedule = models.ForeignKey(RobotSchedule, on_delete=models.CASCADE, related_name='reservations')
    workpoint = models.ForeignKey(WorkPoint, on_delete=models.PROTECT, null=True, blank=True, related_name='reservations')
    resource_type = models.CharField(max_length=40)
    resource_id = models.CharField(max_length=128, db_index=True)
    starts_at = models.DateTimeField(db_index=True)
    ends_at = models.DateTimeField(db_index=True)
    status = models.CharField(max_length=16, choices=STATUS, default='HELD', db_index=True)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['starts_at']
        indexes = [models.Index(fields=['resource_type', 'resource_id', 'status', 'starts_at'])]


class NavigationTag(models.Model):
    warehouse = models.ForeignKey(Warehouse, on_delete=models.CASCADE, related_name='navigation_tags')
    tag_id = models.IntegerField()
    family = models.CharField(max_length=32, default='DATAMATRIX')
    size = models.FloatField(default=0.15)
    floor_id = models.CharField(max_length=64, default='1')
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField(default=0.0)
    yaw = models.FloatField(default=0.0)
    lane_id = models.CharField(max_length=96, blank=True)
    zone = models.ForeignKey(Zone, on_delete=models.SET_NULL, null=True, blank=True, related_name='navigation_tags')
    metadata = models.JSONField(default=dict, blank=True)
    enabled = models.BooleanField(default=True, db_index=True)
    label = models.CharField(max_length=160, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['warehouse_id', 'tag_id']
        constraints = [models.UniqueConstraint(fields=['warehouse', 'tag_id'], name='uniq_navigation_tag_per_warehouse')]


class NavigationTagEdge(models.Model):
    warehouse = models.ForeignKey(Warehouse, on_delete=models.CASCADE, related_name='navigation_edges')
    from_tag = models.ForeignKey(NavigationTag, on_delete=models.CASCADE, related_name='outgoing_edges')
    to_tag = models.ForeignKey(NavigationTag, on_delete=models.CASCADE, related_name='incoming_edges')
    cost = models.FloatField(null=True, blank=True)
    enabled = models.BooleanField(default=True, db_index=True)
    bidirectional = models.BooleanField(default=False)

    class Meta:
        constraints = [models.UniqueConstraint(fields=['warehouse', 'from_tag', 'to_tag'], name='uniq_navigation_edge')]


class RobotNavigationMission(models.Model):
    STATUSES = [(s, s) for s in (
        'PENDING', 'PLANNING', 'NAVIGATING', 'DEAD_RECKONING', 'APPROACH_TAG',
        'TAG_CORRECTION', 'PAUSED', 'ROUTE_DEVIATION', 'TAG_ACQUIRE_FAILED',
        'ARRIVED', 'CANCELLED', 'FAILED', 'EMERGENCY_STOPPED')]
    warehouse = models.ForeignKey(Warehouse, on_delete=models.PROTECT, related_name='tag_missions')
    robot_id = models.CharField(max_length=64, db_index=True)
    target_tag = models.ForeignKey(NavigationTag, on_delete=models.PROTECT, related_name='missions')
    current_tag_id = models.IntegerField(null=True, blank=True)
    next_tag_id = models.IntegerField(null=True, blank=True)
    route = models.JSONField(default=list)
    status = models.CharField(max_length=24, choices=STATUSES, default='PENDING', db_index=True)
    route_index = models.PositiveIntegerField(default=0)
    progress_percent = models.FloatField(default=0.0)
    failure_reason = models.TextField(blank=True)
    started_at = models.DateTimeField(null=True, blank=True)
    updated_at = models.DateTimeField(auto_now=True)
    completed_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ['-id']
        indexes = [models.Index(fields=['warehouse', 'robot_id', 'status'])]
