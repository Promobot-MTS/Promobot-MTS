from django.shortcuts import render, redirect

# Create your views here.
from django.http import HttpResponse, StreamingHttpResponse
from django.template import loader
from django.urls import path
from .robot import RobotWork

from django.contrib.auth.hashers import make_password
from django.contrib.auth.hashers import check_password

count = 1
def index(request):
    # return HttpResponse("Hello, world. You're at the polls index.")
    # global count
    # print("info work--------------------------------------------")
    # count+=1
    latest_question_list=[str(count)]
    return render(
        request,
        'polls/templates/main.html',
        context={'latest_question_list': latest_question_list},
    )

def info(request):
    global count
    print("info work--------------------------------------------")
    count+=1
    latest_question_list=[str(count)]
    return render(
        request,
        'polls/templates/index.html',
        context={'latest_question_list': latest_question_list},
    )

def add(request):
    print("add work--------------------------------------------")
    latest_question_list=[str(count)]
    try:
        pk = request.POST['choice']
        print("pk", pk)
    except Exception as e:
        print(e)


    question_text="test"
    question =["1", "2","3"]
    return render(
        request,
        'polls/templates/add.html',
        context={'latest_question_list': latest_question_list,
                 'question_text': question_text,
                 "question":question},
    )

vote_result ={"1":0, "2":0, "3":0}
votes = 0

from django.http import HttpResponse, HttpResponseRedirect
from django.shortcuts import get_object_or_404, render
from django.urls import reverse

def vote(request):
    global vote_result, votes
    question = ["1", "2", "3"]
    try:
        pk=request.POST['choice']
        vote_result[pk]+=1

    except (KeyError):
        # Redisplay the question voting form.
        return render(request, 'polls/templates/vote.html', {
            'question': question,
            'error_message': "You didn't select a choice.",
        })
    else:
        votes += 1
        print("VOTE OK-----------------------", vote_result)
        return render(request, 'polls/templates/results.html', {
            'question': question,
            "vote_result":vote_result,
            'error_message': "You didn't select a choice.",
        })

        # return HttpResponseRedirect(reverse('polls/templates:results', args=()))


def control(request):
    global vote_result, votes
    try:
        p=request.POST
        g = request.GET
        print("control", p, g)

    except (KeyError):
        pass

    return HttpResponse("ok")



cam = RobotWork()


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


def video(request):
    try:
        # return StreamingHttpResponse(gen(VideoCamera()), content_type="multipart/x-mixed-replace;boundary=frame")
        return StreamingHttpResponse(gen(cam), content_type="multipart/x-mixed-replace;boundary=frame")
    except:  # This is bad! replace it with proper handling
        pass



accounts = {}

def startup():
    accounts.update({"admin": make_password(password="admin", salt=None, hasher="md5")})
    f = open("C:/django/polls/accounts.txt", "r")
    for i in f:
        split = i.split()
        print("split: ", split)
        accounts.update({i[0]: i[1]})
    f.close()

def register(request):
    try:
        login = request.POST['login']
        password = request.POST['password']
        try:
            tmp = accounts[login]
            return render(request, 'polls/templates/register.html', {"error_message": "This login already taken"})
        except:
            if len(password) == 0:
                return render(request, 'polls/templates/register.html', {"error_message", "Password must be not blank"})
            else:
                hash_password = make_password(password=password, salt=None, hasher='md5')
                f = open("C:/django/polls/accounts.txt", "w")
                f.write(str(login) + " " + str(hash_password))
                f.close()
                accounts.update({login: hash_password})
    except:
        return render(request, 'polls/templates/register.html')
    else:
        return redirect('login')


def login(request):
    try:
        login = request.GET['login']
        password = request.GET['password']
        try:
            if check_password(password, accounts[login]):
                return render(request, 'polls/templates/admin.html')
            else:
                return render(request, 'polls/templates/login.html', {"error_message": "login or password error"})
        except:
            return render(request, 'polls/templates/login.html', {"error_messgae": "no account with this login"})
    except:
        return render(request, 'polls/templates/login.html')
