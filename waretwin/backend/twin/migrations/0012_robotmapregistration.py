from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ('twin', '0011_robotvda5050configuration'),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name='RobotMapRegistration',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True,
                                           serialize=False, verbose_name='ID')),
                ('robot_id', models.CharField(db_index=True, max_length=64)),
                ('canonical_revision', models.PositiveIntegerField()),
                ('active_map_id', models.CharField(max_length=128)),
                ('active_map_revision', models.CharField(max_length=128)),
                ('tx', models.FloatField()),
                ('ty', models.FloatField()),
                ('yaw', models.FloatField()),
                ('registration_revision', models.PositiveIntegerField()),
                ('source', models.CharField(max_length=160)),
                ('is_active', models.BooleanField(db_index=True, default=True)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('created_by', models.ForeignKey(blank=True, null=True,
                    on_delete=django.db.models.deletion.SET_NULL, related_name='+',
                    to=settings.AUTH_USER_MODEL)),
                ('warehouse_map', models.ForeignKey(on_delete=django.db.models.deletion.PROTECT,
                    related_name='robot_registrations', to='twin.warehousemap')),
            ],
            options={
                'ordering': ['robot_id', 'active_map_id', 'active_map_revision', '-registration_revision'],
            },
        ),
        migrations.AddConstraint(
            model_name='robotmapregistration',
            constraint=models.UniqueConstraint(
                fields=('robot_id', 'active_map_id', 'active_map_revision', 'registration_revision'),
                name='uniq_robot_map_registration_revision'),
        ),
        migrations.AddConstraint(
            model_name='robotmapregistration',
            constraint=models.UniqueConstraint(
                condition=models.Q(('is_active', True)),
                fields=('robot_id', 'active_map_id', 'active_map_revision'),
                name='uniq_active_robot_map_registration'),
        ),
    ]
