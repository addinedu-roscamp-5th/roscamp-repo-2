document.querySelectorAll('.product-card').forEach(card => {
    card.addEventListener('click', () => {
        alert('상품 상세 페이지로 이동합니다!');
    });
});

document.querySelector('.search-btn').addEventListener('click', () => {
    const query = document.querySelector('.search-input').value;
    if (query.trim()) {
        alert(`"${query}" 검색 결과를 표시합니다!`);
    }
});

