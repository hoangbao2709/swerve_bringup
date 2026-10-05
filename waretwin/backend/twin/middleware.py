import os

class SimpleCorsMiddleware:
    def __init__(self, get_response):
        self.get_response = get_response
        raw = os.getenv('CORS_ALLOWED_ORIGINS', 'http://localhost:5173,http://127.0.0.1:5173')
        self.allowed = {x.strip() for x in raw.split(',') if x.strip()}

    def __call__(self, request):
        from django.http import HttpResponse
        if request.method == 'OPTIONS':
            response = HttpResponse(status=204)
        else:
            response = self.get_response(request)
        origin = request.headers.get('Origin')
        if '*' in self.allowed:
            response['Access-Control-Allow-Origin'] = origin or '*'
        elif origin in self.allowed:
            response['Access-Control-Allow-Origin'] = origin
            response['Vary'] = 'Origin'
        response['Access-Control-Allow-Headers'] = 'Content-Type, X-Layout-Revision'
        response['Access-Control-Allow-Methods'] = 'GET, POST, PUT, PATCH, DELETE, OPTIONS'
        response['Access-Control-Expose-Headers'] = 'X-Warehouse-Id, X-Layout-Revision, X-Layout-Version, X-Layout-Updated-At'
        return response
