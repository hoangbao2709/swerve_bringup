import uuid
from django.db import migrations, models
import django.db.models.deletion
import django.utils.timezone


def seed_inventory_from_existing_load(apps, schema_editor):
    Shelf = apps.get_model('twin', 'Shelf')
    ShelfInventoryItem = apps.get_model('twin', 'ShelfInventoryItem')
    for shelf in Shelf.objects.select_related('zone').all().order_by('id'):
        load = max(0, min(8, int(shelf.current_load or 0)))
        for slot in range(1, load + 1):
            ShelfInventoryItem.objects.create(
                item_uid=f'DEMO-{shelf.id}-{slot}-{uuid.uuid4().hex[:8].upper()}',
                warehouse_id=shelf.zone.warehouse_id,
                shelf_id=shelf.id,
                status='STORED',
                item_code=f'DEMO-{shelf.code}-{slot:02d}',
                item_name=f'Demo item {slot} on {shelf.code}',
                external_ref='',
                quantity=1,
                load_units=1,
                payload_weight_kg=0.0,
                metadata={'seeded_from_shelf_load': True, 'slot': slot},
            )


def clear_seeded_inventory(apps, schema_editor):
    ShelfInventoryItem = apps.get_model('twin', 'ShelfInventoryItem')
    ShelfInventoryItem.objects.filter(metadata__seeded_from_shelf_load=True).delete()


class Migration(migrations.Migration):
    dependencies = [
        ('twin', '0005_shelf_capacity_eight'),
    ]

    operations = [
        migrations.CreateModel(
            name='ShelfInventoryItem',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('item_uid', models.CharField(db_index=True, max_length=64, unique=True)),
                ('status', models.CharField(choices=[('STORED', 'STORED'), ('RESERVED', 'RESERVED'), ('OUTBOUND', 'OUTBOUND')], db_index=True, default='STORED', max_length=16)),
                ('item_code', models.CharField(blank=True, db_index=True, max_length=128)),
                ('item_name', models.CharField(blank=True, max_length=200)),
                ('external_ref', models.CharField(blank=True, db_index=True, max_length=128)),
                ('quantity', models.PositiveIntegerField(default=1)),
                ('load_units', models.PositiveIntegerField(default=1)),
                ('payload_weight_kg', models.FloatField(default=0.0)),
                ('metadata', models.JSONField(blank=True, default=dict)),
                ('stored_at', models.DateTimeField(default=django.utils.timezone.now)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('origin_order', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='originated_inventory_items', to='twin.warehouseorder')),
                ('last_movement_order', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='last_moved_inventory_items', to='twin.warehouseorder')),
                ('reserved_by_order', models.OneToOneField(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='reserved_inventory_item', to='twin.warehouseorder')),
                ('shelf', models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='inventory_items', to='twin.shelf')),
                ('warehouse', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='inventory_items', to='twin.warehouse')),
            ],
            options={
                'ordering': ['shelf_id', 'stored_at', 'id'],
            },
        ),
        migrations.AddIndex(
            model_name='shelfinventoryitem',
            index=models.Index(fields=['warehouse', 'shelf', 'status'], name='twin_shelfi_warehou_97101e_idx'),
        ),
        migrations.AddIndex(
            model_name='shelfinventoryitem',
            index=models.Index(fields=['warehouse', 'item_code', 'status'], name='twin_shelfi_warehou_022fde_idx'),
        ),
        migrations.RunPython(seed_inventory_from_existing_load, clear_seeded_inventory),
    ]
