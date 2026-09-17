from accounts.models import ApiToken, role_of

def token_from_request(request):
    auth = request.headers.get('Authorization', '')
    if auth.lower().startswith('bearer '):
        return auth.split(' ', 1)[1].strip()
    return request.GET.get('token')

def user_from_token(token):
    if not token:
        return None
    try:
        obj = ApiToken.objects.select_related('user', 'user__waretwin_profile').get(key=token)
    except ApiToken.DoesNotExist:
        return None
    if not obj.user.is_active:
        return None
    obj.user.waretwin_role = role_of(obj.user)
    return obj.user

def user_from_request(request):
    return user_from_token(token_from_request(request))
