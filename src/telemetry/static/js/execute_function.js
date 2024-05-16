document.querySelectorAll(".remote_func").forEach((btn) => {
    btn.onclick = () => {
        return fetch(`/execute_function/${btn.id}`, {
            method: "POST"
        });
    };
});
