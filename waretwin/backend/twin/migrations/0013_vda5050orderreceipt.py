from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('twin', '0012_robotmapregistration'),
    ]

    operations = [
        migrations.CreateModel(
            name='Vda5050OrderReceipt',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True,
                                           serialize=False, verbose_name='ID')),
                ('robot_id', models.CharField(max_length=64)),
                ('order_id', models.CharField(max_length=128)),
                ('latest_update_id', models.PositiveBigIntegerField()),
                ('payload_sha256', models.CharField(max_length=64)),
                ('route_status', models.CharField(default='CLAIMED', max_length=24)),
                ('received_at', models.DateTimeField(auto_now=True)),
            ],
        ),
        migrations.AddConstraint(
            model_name='vda5050orderreceipt',
            constraint=models.UniqueConstraint(
                fields=('robot_id', 'order_id'), name='uniq_vda_order_robot_order'),
        ),
    ]
