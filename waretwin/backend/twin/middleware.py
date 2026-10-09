from django.conf import settings

class SimpleCorsMiddleware:
    def __init__(self, get_response):
        self.get_response = get_response
        self.allowed = set(getattr(settings, 'WARETWIN_OPERATOR_ALLOWED_ORIGINS', ()))

    def __call__(self, request):
        from django.http import HttpResponse
        if request.method == 'OPTIONS':
            response = HttpResponse(status=204)
        else:
            response = self.get_response(request)
        origin = request.headers.get('Origin')
        if origin in self.allowed:
            response['Access-Control-Allow-Origin'] = origin
            response['Vary'] = 'Origin'
            response['Access-Control-Allow-Headers'] = 'Content-Type, X-Layout-Revision'
            response['Access-Control-Allow-Methods'] = 'GET, POST, PUT, PATCH, DELETE, OPTIONS'
            response['Access-Control-Expose-Headers'] = 'X-Warehouse-Id, X-Layout-Revision, X-Layout-Version, X-Layout-Updated-At'
        response['X-Content-Type-Options'] = 'nosniff'
        response['X-Frame-Options'] = 'DENY'
        response['Referrer-Policy'] = 'no-referrer'
        if request.path.startswith('/api/'):
            response['Cache-Control'] = 'no-store'
        return response
