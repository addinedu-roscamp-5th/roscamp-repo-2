updateCartSummary()
// 수량 변경
function updateQuantity(btn, change) {
    const input = btn.parentNode.querySelector('.quantity-input');
    let currentValue = parseInt(input.value);
    let newValue = currentValue + change;
    
    if (newValue >= 1 && newValue <= 99) {
        input.value = newValue;
        updateCartSummary();
    }
}

// 아이템 삭제
function removeItem(btn) {
    if (confirm('이 상품을 장바구니에서 삭제하시겠습니까?')) {
        btn.closest('.cart-item').remove();
        updateCartCount();
        updateCartSummary();
    }
}


function updateCartCount() {
    const cartItems = document.querySelectorAll('.cart-item');
    const count = cartItems.length;
    const cartCountElem = document.querySelector('.cart-count');
    if (cartCountElem) {
        cartCountElem.textContent = count;
    }
}

// 페이지 로드 후 자동 실행
document.addEventListener("DOMContentLoaded", () => {
    updateCartCount();
    updateCartSummary();
    updateSelectAllCount(); // 숫자 라벨 업데이트 

    // 수량 변경 후 수동 업데이트도 반영 가능 (옵션)
    const quantityInputs = document.querySelectorAll('.quantity-input');
    quantityInputs.forEach(input => {
        input.addEventListener('change', updateCartCount);
    });
});

// 장바구니 요약 업데이트
function updateCartSummary() {
    let totalPrice = 0;
    let discount = 0;

    const cartItems = document.querySelectorAll('.cart-item');
    cartItems.forEach(item => {
        const checkbox = item.querySelector('.item-checkbox');
        if (!checkbox || !checkbox.checked) return; // 체크된 상품만 합산

        const quantity = parseInt(item.querySelector('.quantity-input').value);
        const priceText = item.querySelector('.current-price').textContent;
        const originalText = item.querySelector('.original-price')?.textContent;

        const price = parseInt(priceText.replace(/[^0-9]/g, ''));
        const original = originalText ? parseInt(originalText.replace(/[^0-9]/g, '')) : price;

        if (!isNaN(quantity) && !isNaN(price)) {
            totalPrice += quantity * price;
            discount += quantity * (original - price);
        }
    });

    // DOM에 적용
    const totalPriceText = totalPrice.toLocaleString('ko-KR') + '원';
    const discountText = '-' + discount.toLocaleString('ko-KR') + '원';
    const finalPriceText = (totalPrice).toLocaleString('ko-KR') + '원';

    const updateElem = (selector, value) => {
        const el = document.querySelector(selector);
        if (el) el.textContent = value;
    };

    updateElem('#total-price', totalPriceText);
    updateElem('#discount-price', discountText);
    updateElem('#final-price', finalPriceText);

    const orderBtn = document.querySelector('.checkout-btn');
    if (orderBtn) {
        orderBtn.innerHTML = `💳 주문하기 (${finalPriceText})`;
    }
}

// 전체 선택/해제
document.getElementById('select-all').addEventListener('change', function() {
    const checkboxes = document.querySelectorAll('.item-checkbox');
    checkboxes.forEach(checkbox => {
        checkbox.checked = this.checked;
    });
    updateCartSummary(); // 전체 선택/해제 시 요약 갱신
    updateSelectAllCount(); // 추가: 숫자 라벨 업데이트
});

// 체크박스/수량 변경 시 요약 자동 갱신 및 전체선택 상태 동기화
document.addEventListener('change', function(e) {
    if (
        e.target.classList.contains('item-checkbox') ||
        e.target.classList.contains('quantity-input')
    ) {
        updateCartSummary();
    }
    if (e.target.classList.contains('item-checkbox')) {
        const allCheckboxes = document.querySelectorAll('.item-checkbox');
        const checkedCheckboxes = document.querySelectorAll('.item-checkbox:checked');
        const selectAllCheckbox = document.getElementById('select-all');
        if (selectAllCheckbox) {
            selectAllCheckbox.checked = allCheckboxes.length === checkedCheckboxes.length;
        }
        updateSelectAllCount(); // 숫자 라벨 업데이트 
    }
});

function updateSelectAllCount() {
    const checkedCount = document.querySelectorAll('.item-checkbox:checked').length; // 선택된 개수
    const countLabel = document.querySelector('.select-all-count'); // 숫자 표시용 라벨
    if (countLabel) {
        countLabel.textContent = `(${checkedCount}개)`;
    }
}


// 결제 진행
function proceedToCheckout() {
    const items = [];

    document.querySelectorAll(".cart-item").forEach(item => {
        // const product_name = item.querySelector(".product-name").textContent.trim();
        // const cartItems = document.querySelectorAll(".cart-item");
        const quantity = parseInt(item.querySelector(".quantity-input").value);
        const price = parseInt(item.querySelector(".product-price").textContent.replace(/[^0-9]/g, ""));
        const itemId = item.dataset.id;
        items.push({ itemId, quantity, price });
        console.log(items);
        
    });

    fetch("/orders", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ items })
    })
    .then(res => res.json())
    .then(data => {
        if (data.status === "success") {
            // ✅ 결제 완료 페이지로 이동
            window.location.href = "/payment-complete";
        } else {
            alert("❌ 주문 처리 실패");
        }
    })
    .catch(err => {
        console.error("Error:", err);
        alert("❌ 주문 처리 중 오류 발생");
    });
}


// 쇼핑 계속하기
document.querySelector('.continue-shopping').addEventListener('click', function(e) {
    e.preventDefault();
    alert('쇼핑몰 메인 페이지로 이동합니다.');
    // 실제로는 메인 페이지로 리다이렉트
});