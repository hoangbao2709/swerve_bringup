from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [
        ('auth', '0012_alter_user_first_name_max_length'),
        ('twin', '0003_warehouse_map'),
    ]

    operations = [
        migrations.CreateModel(
            name='WorkPoint',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('code', models.CharField(max_length=96)), ('name', models.CharField(max_length=180)),
                ('kind', models.CharField(choices=[('SHELF','SHELF'),('CONVEYOR_IN','CONVEYOR_IN'),('CONVEYOR_OUT','CONVEYOR_OUT'),('INBOUND','INBOUND'),('OUTBOUND','OUTBOUND'),('PACKING','PACKING'),('SORTING','SORTING'),('BUFFER','BUFFER'),('CHARGING','CHARGING'),('DOCK','DOCK'),('STATION','STATION'),('PARKING','PARKING'),('CUSTOM','CUSTOM')], db_index=True, max_length=24)),
                ('floor', models.PositiveIntegerField(db_index=True, default=1)), ('x', models.FloatField(default=0.0)), ('y', models.FloatField(default=0.0)), ('z', models.FloatField(default=0.0)), ('yaw', models.FloatField(default=0.0)),
                ('resource_type', models.CharField(blank=True, max_length=40)), ('resource_id', models.CharField(blank=True, db_index=True, max_length=128)), ('enabled', models.BooleanField(db_index=True, default=True)), ('capacity', models.PositiveIntegerField(default=1)), ('metadata', models.JSONField(blank=True, default=dict)),
                ('created_at', models.DateTimeField(auto_now_add=True)), ('updated_at', models.DateTimeField(auto_now=True)),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='workpoints', to='twin.warehouse')),
                ('zone', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='workpoints', to='twin.zone')),
            ],
            options={'ordering':['warehouse_id','kind','code']},
        ),
        migrations.CreateModel(
            name='RobotProfile',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('robot_id', models.CharField(max_length=64)), ('name', models.CharField(blank=True, max_length=160)), ('enabled', models.BooleanField(db_index=True, default=True)), ('payload_capacity', models.PositiveIntegerField(default=4)), ('min_dispatch_battery', models.FloatField(default=20.0)), ('capabilities', models.JSONField(blank=True, default=list)), ('metadata', models.JSONField(blank=True, default=dict)), ('created_at', models.DateTimeField(auto_now_add=True)), ('updated_at', models.DateTimeField(auto_now=True)),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='robot_profiles', to='twin.warehouse')),
            ], options={'ordering':['warehouse_id','robot_id']},
        ),
        migrations.CreateModel(
            name='WarehouseOrder',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('order_no', models.CharField(db_index=True, max_length=64, unique=True)), ('external_ref', models.CharField(blank=True, db_index=True, max_length=128)),
                ('type', models.CharField(choices=[('MOVE','MOVE'),('PICK','PICK'),('REPLENISH','REPLENISH'),('INBOUND','INBOUND'),('OUTBOUND','OUTBOUND'),('TRANSFER','TRANSFER')], db_index=True, default='MOVE', max_length=20)),
                ('priority', models.CharField(choices=[('LOW','LOW'),('NORMAL','NORMAL'),('HIGH','HIGH'),('CRITICAL','CRITICAL')], db_index=True, default='NORMAL', max_length=16)),
                ('status', models.CharField(choices=[('NEW','NEW'),('PLANNED','PLANNED'),('QUEUED','QUEUED'),('RUNNING','RUNNING'),('COMPLETED','COMPLETED'),('CANCELLED','CANCELLED'),('FAILED','FAILED')], db_index=True, default='NEW', max_length=16)),
                ('quantity', models.PositiveIntegerField(default=1)), ('load_units', models.PositiveIntegerField(default=1)), ('payload_weight_kg', models.FloatField(default=0.0)), ('due_at', models.DateTimeField(blank=True, db_index=True, null=True)), ('notes', models.TextField(blank=True)), ('metadata', models.JSONField(blank=True, default=dict)), ('created_at', models.DateTimeField(auto_now_add=True)), ('updated_at', models.DateTimeField(auto_now=True)),
                ('created_by', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='+', to='auth.user')),
                ('destination', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='orders_to', to='twin.workpoint')),
                ('source', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='orders_from', to='twin.workpoint')),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='orders', to='twin.warehouse')),
            ], options={'ordering':['-created_at']},
        ),
        migrations.CreateModel(
            name='RobotSchedule',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('schedule_id', models.CharField(db_index=True, max_length=64, unique=True)), ('mode', models.CharField(choices=[('AUTO','AUTO'),('SEMI_AUTO','SEMI_AUTO'),('MANUAL','MANUAL')], default='AUTO', max_length=16)), ('status', models.CharField(choices=[('DRAFT','DRAFT'),('PLANNED','PLANNED'),('QUEUED','QUEUED'),('RUNNING','RUNNING'),('COMPLETED','COMPLETED'),('FAILED','FAILED'),('CANCELLED','CANCELLED')], db_index=True, default='PLANNED', max_length=16)),
                ('planned_start', models.DateTimeField(db_index=True)), ('planned_end', models.DateTimeField(blank=True, null=True)), ('actual_start', models.DateTimeField(blank=True, null=True)), ('actual_end', models.DateTimeField(blank=True, null=True)), ('estimated_distance_m', models.FloatField(default=0.0)), ('estimated_duration_s', models.FloatField(default=0.0)), ('score', models.JSONField(blank=True, default=dict)), ('current_leg', models.PositiveIntegerField(default=0)), ('engine_task_id', models.CharField(blank=True, db_index=True, max_length=64)), ('failure_reason', models.TextField(blank=True)), ('created_at', models.DateTimeField(auto_now_add=True)), ('updated_at', models.DateTimeField(auto_now=True)),
                ('created_by', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='+', to='auth.user')),
                ('order', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='schedules', to='twin.warehouseorder')),
                ('robot', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='schedules', to='twin.robotprofile')),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='robot_schedules', to='twin.warehouse')),
            ], options={'ordering':['planned_start','schedule_id']},
        ),
        migrations.CreateModel(
            name='ScheduleStop',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('sequence', models.PositiveIntegerField()), ('action', models.CharField(choices=[('PICKUP','PICKUP'),('DROPOFF','DROPOFF'),('TRANSIT','TRANSIT'),('WAIT','WAIT'),('CHARGE','CHARGE')], default='TRANSIT', max_length=16)), ('service_seconds', models.PositiveIntegerField(default=0)), ('status', models.CharField(choices=[('PENDING','PENDING'),('ACTIVE','ACTIVE'),('COMPLETED','COMPLETED'),('SKIPPED','SKIPPED'),('FAILED','FAILED')], default='PENDING', max_length=16)), ('arrived_at', models.DateTimeField(blank=True, null=True)), ('completed_at', models.DateTimeField(blank=True, null=True)),
                ('schedule', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='stops', to='twin.robotschedule')),
                ('workpoint', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='schedule_stops', to='twin.workpoint')),
            ], options={'ordering':['schedule_id','sequence']},
        ),
        migrations.CreateModel(
            name='ResourceReservation',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('resource_type', models.CharField(max_length=40)), ('resource_id', models.CharField(db_index=True, max_length=128)), ('starts_at', models.DateTimeField(db_index=True)), ('ends_at', models.DateTimeField(db_index=True)), ('status', models.CharField(choices=[('HELD','HELD'),('ACTIVE','ACTIVE'),('RELEASED','RELEASED'),('CANCELLED','CANCELLED')], db_index=True, default='HELD', max_length=16)), ('created_at', models.DateTimeField(auto_now_add=True)),
                ('schedule', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='reservations', to='twin.robotschedule')),
                ('workpoint', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.PROTECT, related_name='reservations', to='twin.workpoint')),
            ], options={'ordering':['starts_at']},
        ),
        migrations.AddConstraint(model_name='workpoint', constraint=models.UniqueConstraint(fields=('warehouse','code'), name='uniq_workpoint_code_per_warehouse')),
        migrations.AddIndex(model_name='workpoint', index=models.Index(fields=['warehouse','kind','enabled'], name='twin_workpo_warehou_69518d_idx')),
        migrations.AddConstraint(model_name='robotprofile', constraint=models.UniqueConstraint(fields=('warehouse','robot_id'), name='uniq_robot_profile_per_warehouse')),
        migrations.AddIndex(model_name='warehouseorder', index=models.Index(fields=['warehouse','status','priority'], name='twin_wareho_warehou_9ba0f8_idx')),
        migrations.AddIndex(model_name='robotschedule', index=models.Index(fields=['warehouse','status','planned_start'], name='twin_robots_warehou_c3d031_idx')),
        migrations.AddIndex(model_name='robotschedule', index=models.Index(fields=['robot','status','planned_start'], name='twin_robots_robot_i_0c0ee2_idx')),
        migrations.AddConstraint(model_name='schedulestop', constraint=models.UniqueConstraint(fields=('schedule','sequence'), name='uniq_schedule_stop_sequence')),
        migrations.AddIndex(model_name='resourcereservation', index=models.Index(fields=['resource_type','resource_id','status','starts_at'], name='twin_resour_resourc_79e93a_idx')),
    ]
