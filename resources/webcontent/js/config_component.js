class ConfigComponent extends HTMLElement {


    /**
     * Represents a custom element for displaying and interacting with configuration data.
     */
    constructor() {
        super();

        this.render();
    }

    /**
     * Returns the CSS styles for the component.
     * @returns {string} The CSS styles as a string.
     */
    css() {
        return ` 
            * {
                font-family: sans-serif
            }
        
            #yamlData {
                font-size: 1rem
            }
            
            #yamlData div {
                margin-bottom: 3px;
            }
            
            #yamlData div div {
                margin-left: 2ch;
                margin-bottom: 0.5rem;
                margin-top: 0.5rem;
            }        
            
            .submit-btn {
                margin-left: 5px;
            }       
        `
    }

    /**
     * Renders the component.
     */
    render() {
        this.shadow = this.attachShadow({mode: "closed"})
        this.shadow.innerHTML = `<div>
                    <h2>Config</h2>
                    <div>
                        <button onclick="this.saveConfig()">save</button>
                        <button onclick="this.rollbackConfig()">rollback</button>
                    </div>
                    <div id="yamlData"></div>
                </div>`

        const styleTag = document.createElement("style")
        styleTag.innerHTML = this.css()

        this.shadow.appendChild(styleTag)
        this.getConfig();
    }

    /**
     * Fetches the structure of the configuration.
     *
     * @returns {Promise<Object>} A promise that resolves with the configuration structure.
     */
    async getConfigStructure() {
        return fetch("/get-config-structure")
            .then(res => res.json())
    }

    /**
     * Sends a request to save the current configuration.
     */
    async saveConfig() {
        await fetch("./store-config", {method: "POST"})
    }

    /**
     * Sends a request to roll back the configuration to its previous state.
     */
    async rollbackConfig() {
        await fetch("./rollback-config", {method: "POST"})
        await this.getConfig()
    }

    /**
     * Fetches the current configuration data and renders it.
     */
    async getConfig() {
        const response = await fetch("/get-config");
        if (response.ok) {
            const jsonData = await response.json();
            const structure = await this.getConfigStructure()
            this.renderKeysAndInputs(jsonData, structure);
        } else {
            console.error("Error fetching configuration");
        }
    }

    /**
     * Renders keys and inputs for the configuration data.
     *
     * @param {Object} obj - The configuration data object.
     * @param {Object} structure - The structure of the configuration.
     * @param {string} [parentKey="yamlData"] - The parent key for rendering.
     * @param {number} [depth=0] - The depth of rendering.
     */
    renderKeysAndInputs(obj, structure, parentKey = "yamlData", depth = 0) {
        const parent = this.shadow.getElementById(parentKey)
        parent.innerHTML = ""
        parent.style.marginBottom = "1.5em"
        for (const key in obj) {
            if (Object.hasOwnProperty.call(obj, key)) {
                const value = obj[key];
                const struc = structure[key]

                let currentKey = parentKey ? `${parentKey}.${key}` : key;

                const div = document.createElement("div");
                div.className = "input-container";
                this.shadow.getElementById(parentKey).appendChild(div);

                if (typeof value === "object") {
                    div.innerHTML = `<div>${key}: <div id="${currentKey}"></div></div>`;
                    this.renderKeysAndInputs(value, struc, currentKey, depth + 1);
                    continue
                }

                let inputType = "text";
                let stepValue = "";

                switch (struc) {
                    case "float":
                        stepValue = "0.0001";
                    /* FALLTHROUGH */
                    case "int":
                        inputType = "number"
                        break

                    case "boolean":
                        inputType = "checkbox"
                        break
                    case "str":
                        inputType = "text"
                        break
                }

                currentKey = currentKey.replace("yamlData.", "")

                div.innerHTML = `
                <div class="input-wrapper">
                    <label for="${currentKey}">${key}:</label> 
                    <input type="${inputType}" id="${currentKey}" name="${currentKey}" value="${value}" step="${stepValue}">
                    <button class="submit-btn" onclick="submit('${currentKey}')">Submit</button>
                </div>
            `;
            }
        }
    }

    /**
     * Submits a new value for a configuration key.
     *
     * @param {string} key - The key of the configuration to update.
     */
    async submit(key) {
        const inputElement = this.shadow.getElementById(key);
        const newValue = inputElement.type === "number"
            ? parseFloat(inputElement.value)
            : inputElement.type === "checkbox"
                ? inputElement.checked
                : inputElement.value

        const response = await fetch("/update-config", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                key: key,
                value: newValue
            })
        });
    }
}

customElements.define("config-shower", ConfigComponent);
