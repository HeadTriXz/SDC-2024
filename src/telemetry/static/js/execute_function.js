document.querySelectorAll(".remote_func").forEach((btn) => {
    btn.innerHTML = btn.id;
    btn.onclick = () => {
        return fetch(`/execute_function/${btn.id}`, {
            method: "POST"
        });
    };
});
