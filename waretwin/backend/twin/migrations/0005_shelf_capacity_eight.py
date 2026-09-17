from django.db import migrations, models


def normalize_existing_shelves(apps, schema_editor):
    Shelf = apps.get_model('twin', 'Shelf')
    shelves = list(Shelf.objects.order_by('zone_id', 'code', 'id'))
    if not shelves:
        return

    # Capacity is a product invariant from this version onward.
    Shelf.objects.all().update(capacity=8)

    # Upgrade old demo databases without overwriting real operational data:
    # only seed the light 0/1/2/3 pattern when every existing shelf is empty.
    if Shelf.objects.exclude(current_load=0).exists():
        return

    starter_pattern = (0, 1, 0, 2, 0, 1, 0, 3)
    for index, shelf in enumerate(shelves):
        load = starter_pattern[index % len(starter_pattern)]
        shelf.current_load = load
        if shelf.status == 'AVAILABLE' and load > 0:
            shelf.status = 'OCCUPIED'
        shelf.save(update_fields=['current_load', 'status'])


class Migration(migrations.Migration):
    dependencies = [
        ('twin', '0004_scheduler_orders_workpoints'),
    ]

    operations = [
        migrations.AlterField(
            model_name='shelf',
            name='capacity',
            field=models.PositiveIntegerField(default=8),
        ),
        migrations.RunPython(normalize_existing_shelves, migrations.RunPython.noop),
    ]
