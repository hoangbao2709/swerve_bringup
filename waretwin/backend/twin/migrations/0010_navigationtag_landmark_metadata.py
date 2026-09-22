from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ('twin', '0009_navigationtag_navigationtagedge_and_more'),
    ]

    operations = [
        migrations.AddField(
            model_name='navigationtag',
            name='family',
            field=models.CharField(default='DATAMATRIX', max_length=32),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='size',
            field=models.FloatField(default=0.15),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='floor_id',
            field=models.CharField(default='1', max_length=64),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='z',
            field=models.FloatField(default=0.0),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='lane_id',
            field=models.CharField(blank=True, max_length=96),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='metadata',
            field=models.JSONField(blank=True, default=dict),
        ),
        migrations.AddField(
            model_name='navigationtag',
            name='zone',
            field=models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='navigation_tags', to='twin.zone'),
        ),
    ]
