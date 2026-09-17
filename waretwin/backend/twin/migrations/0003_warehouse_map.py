from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [
        ('twin', '0002_warehouse_zone_shelf'),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name='WarehouseMap',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('layout', models.JSONField(default=dict)),
                ('draft', models.JSONField(default=dict)),
                ('revision', models.PositiveIntegerField(db_index=True, default=1)),
                ('published_version', models.PositiveIntegerField(default=0)),
                ('is_active', models.BooleanField(db_index=True, default=False)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('updated_by', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='+', to=settings.AUTH_USER_MODEL)),
                ('warehouse', models.OneToOneField(on_delete=django.db.models.deletion.CASCADE, related_name='map', to='twin.warehouse')),
            ],
            options={'ordering': ['warehouse_id']},
        ),
        migrations.CreateModel(
            name='WarehouseMapVersion',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('version', models.PositiveIntegerField()),
                ('revision', models.PositiveIntegerField()),
                ('layout', models.JSONField(default=dict)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('created_by', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='+', to=settings.AUTH_USER_MODEL)),
                ('warehouse_map', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='versions', to='twin.warehousemap')),
            ],
            options={'ordering': ['-version']},
        ),
        migrations.AddConstraint(
            model_name='warehousemapversion',
            constraint=models.UniqueConstraint(fields=('warehouse_map', 'version'), name='uniq_map_version_per_warehouse'),
        ),
    ]
