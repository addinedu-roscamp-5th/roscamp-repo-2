from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from database import database
from routes import orders


app = FastAPI()

app.include_router(orders.router)

app.mount("/static", StaticFiles(directory="static"), name="static")

# templates 디렉토리 등록
templates = Jinja2Templates(directory="templates")


# 루트 경로: 메인 쇼핑몰 페이지
@app.get("/", response_class=HTMLResponse)
async def bolt_mall(request: Request):
    return templates.TemplateResponse("bolt-mall.html", {"request": request})

# 장바구니 페이지
@app.get("/cart", response_class=HTMLResponse)
async def bolt_cart(request: Request):
    return templates.TemplateResponse("bolt-mall-cart.html", {"request": request})

@app.get("/payment-complete", response_class=HTMLResponse)
async def payment_complete(request: Request):
    return templates.TemplateResponse("payment-complete.html", {"request": request})

