from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [('twin', '0001_initial')]

    operations = [
        migrations.CreateModel(
            name='Warehouse',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('code', models.CharField(db_index=True, max_length=64, unique=True)),
                ('name', models.CharField(max_length=160)),
                ('description', models.TextField(blank=True)),
                ('status', models.CharField(choices=[('ACTIVE','ACTIVE'),('INACTIVE','INACTIVE')], db_index=True, default='ACTIVE', max_length=16)),
                ('width', models.FloatField(default=1.0)),
                ('depth', models.FloatField(default=1.0)),
                ('height', models.FloatField(default=1.0)),
                ('units', models.CharField(default='m', max_length=16)),
                ('layout_id', models.CharField(blank=True, db_index=True, max_length=128)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
            ],
            options={'ordering': ['code']},
        ),
        migrations.CreateModel(
            name='Zone',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('code', models.CharField(max_length=64)),
                ('name', models.CharField(max_length=160)),
                ('type', models.CharField(choices=[('STORAGE','STORAGE'),('PICKING','PICKING'),('BUFFER','BUFFER'),('CHARGING','CHARGING'),('RESTRICTED','RESTRICTED'),('OTHER','OTHER')], db_index=True, default='STORAGE', max_length=20)),
                ('status', models.CharField(choices=[('ACTIVE','ACTIVE'),('INACTIVE','INACTIVE'),('BLOCKED','BLOCKED')], db_index=True, default='ACTIVE', max_length=16)),
                ('floor', models.PositiveIntegerField(db_index=True, default=1)),
                ('color', models.CharField(default='#3b82f6', max_length=16)),
                ('polygon', models.JSONField(blank=True, default=list)),
                ('description', models.TextField(blank=True)),
                ('layout_zone_id', models.CharField(blank=True, db_index=True, max_length=128)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='zones', to='twin.warehouse')),
            ],
            options={'ordering': ['warehouse_id', 'floor', 'code']},
        ),
        migrations.CreateModel(
            name='Shelf',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('code', models.CharField(max_length=64)),
                ('name', models.CharField(max_length=160)),
                ('type', models.CharField(choices=[('STORAGE','STORAGE'),('PICK_FACE','PICK_FACE'),('BUFFER','BUFFER'),('OTHER','OTHER')], db_index=True, default='STORAGE', max_length=20)),
                ('status', models.CharField(choices=[('AVAILABLE','AVAILABLE'),('OCCUPIED','OCCUPIED'),('FULL','FULL'),('RESERVED','RESERVED'),('DISABLED','DISABLED'),('MAINTENANCE','MAINTENANCE')], db_index=True, default='AVAILABLE', max_length=20)),
                ('floor', models.PositiveIntegerField(db_index=True, default=1)),
                ('position_x', models.FloatField(default=0.0)), ('position_y', models.FloatField(default=0.0)), ('position_z', models.FloatField(default=0.0)),
                ('width', models.FloatField(default=1.0)), ('depth', models.FloatField(default=1.0)), ('height', models.FloatField(default=1.0)), ('rotation_deg', models.FloatField(default=0.0)), ('levels', models.PositiveIntegerField(default=1)),
                ('capacity', models.PositiveIntegerField(default=1)), ('current_load', models.PositiveIntegerField(default=0)),
                ('access_x', models.FloatField(default=0.0)), ('access_y', models.FloatField(default=0.0)), ('access_yaw', models.FloatField(default=0.0)),
                ('layout_rack_id', models.CharField(blank=True, db_index=True, max_length=128)),
                ('description', models.TextField(blank=True)), ('metadata', models.JSONField(blank=True, default=dict)),
                ('created_at', models.DateTimeField(auto_now_add=True)), ('updated_at', models.DateTimeField(auto_now=True)),
                ('zone', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name='shelves', to='twin.zone')),
            ],
            options={'ordering': ['zone_id', 'code']},
        ),
        migrations.AddConstraint(model_name='zone', constraint=models.UniqueConstraint(fields=('warehouse','code'), name='uniq_zone_code_per_warehouse')),
        migrations.AddConstraint(model_name='shelf', constraint=models.UniqueConstraint(fields=('zone','code'), name='uniq_shelf_code_per_zone')),
    ]
