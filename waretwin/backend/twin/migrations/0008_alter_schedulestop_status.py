from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [('twin', '0007_rename_twin_eventl_run_id_7dce70_idx_twin_eventl_run_id_d614e1_idx_and_more')]

    operations = [
        migrations.AlterField(
            model_name='schedulestop',
            name='status',
            field=models.CharField(
                choices=[
                    ('PENDING', 'PENDING'), ('ACTIVE', 'ACTIVE'),
                    ('ARRIVED', 'ARRIVED'), ('COMPLETED', 'COMPLETED'),
                    ('SKIPPED', 'SKIPPED'), ('FAILED', 'FAILED'),
                ],
                default='PENDING', max_length=16,
            ),
        ),
    ]
