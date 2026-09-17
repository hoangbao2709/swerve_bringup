from django.db import migrations, models

class Migration(migrations.Migration):
    initial = True
    dependencies = []
    operations = [
        migrations.CreateModel(
            name='EventLog',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('run_id', models.CharField(db_index=True, max_length=32)),
                ('event_id', models.CharField(max_length=64)),
                ('tick', models.IntegerField(db_index=True, default=0)),
                ('type', models.CharField(db_index=True, max_length=64)),
                ('source', models.CharField(blank=True, max_length=32)),
                ('severity', models.CharField(db_index=True, max_length=16)),
                ('message', models.TextField(blank=True)),
                ('robot_id', models.CharField(blank=True, db_index=True, max_length=32, null=True)),
                ('task_id', models.CharField(blank=True, max_length=32, null=True)),
                ('zone_id', models.CharField(blank=True, db_index=True, max_length=32, null=True)),
                ('conveyor_id', models.CharField(blank=True, max_length=32, null=True)),
                ('camera_id', models.CharField(blank=True, max_length=32, null=True)),
                ('payload', models.JSONField(blank=True, default=dict)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
            ],
            options={'ordering': ['-id']},
        ),
        migrations.AddIndex(model_name='eventlog', index=models.Index(fields=['run_id','-tick'], name='twin_eventl_run_id_7dce70_idx')),
        migrations.CreateModel(
            name='RobotEndpoint',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('robot_id', models.CharField(max_length=32, unique=True)),
                ('name', models.CharField(blank=True, max_length=128)),
                ('base_url', models.URLField(blank=True)),
                ('ws_url', models.URLField(blank=True)),
                ('enabled', models.BooleanField(default=True)),
                ('last_seen', models.DateTimeField(blank=True, null=True)),
                ('metadata', models.JSONField(blank=True, default=dict)),
            ],
        ),
        migrations.CreateModel(
            name='Mission',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('mission_id', models.CharField(max_length=64, unique=True)),
                ('robot_id', models.CharField(blank=True, max_length=32, null=True)),
                ('kind', models.CharField(default='NAVIGATE', max_length=32)),
                ('target', models.JSONField(default=dict)),
                ('status', models.CharField(choices=[('PENDING','PENDING'),('SENT','SENT'),('RUNNING','RUNNING'),('DONE','DONE'),('FAILED','FAILED'),('CANCELLED','CANCELLED')], default='PENDING', max_length=16)),
                ('command_id', models.CharField(blank=True, max_length=64)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
            ],
        ),
    ]
