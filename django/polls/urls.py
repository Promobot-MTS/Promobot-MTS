from django.urls import path

from . import views

views.startup()

urlpatterns = [
    path('', views.index, name='index'),
    path('info/', views.info, name='info'),
    path('add/', views.add, name='add'),
    path('vote/', views.vote, name='vote'),
    path('video/', views.video, name='video'),
    path('control/', views.control, name='control'),
    path('login/', views.login, name='login'),
    path('register/', views.register, name='register'),
]