#ifndef RL_POLICY_JSON_H
#define RL_POLICY_JSON_H

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <map>
#include <set>

// Simple JSON parser (minimal implementation)
// For production, consider using nlohmann/json library
namespace json {
    class Value;
    class Parser;
}

class RLPolicyJSON {
public:
    enum class Activation {
        RELU,
        ELU,
        TANH
    };

    struct LinearLayer {
        std::vector<std::vector<float>> weights;  // [out_features, in_features]
        std::vector<float> bias;                   // [out_features]
        int in_features;
        int out_features;
    };

    struct EncoderLayer {
        LinearLayer linear;
        Activation activation;
    };

    struct GRUWeights {
        // Weight matrices: [3 * hidden_size, input_size] for input-to-hidden
        //                  [3 * hidden_size, hidden_size] for hidden-to-hidden
        std::vector<std::vector<float>> weight_ih;  // [3*hidden, input]
        std::vector<std::vector<float>> weight_hh;  // [3*hidden, hidden]
        std::vector<float> bias_ih;                 // [3*hidden]
        std::vector<float> bias_hh;                 // [3*hidden]
        int input_size;
        int hidden_size;
    };

    RLPolicyJSON();
    ~RLPolicyJSON();

    // Load policy from JSON file
    bool load_from_json(const std::string& json_path, Activation nonlinearity = Activation::RELU);

    // Forward pass: returns [mean, logstd] concatenated
    std::vector<float> forward(const std::vector<float>& obs, bool normalized = false);

    // Reset hidden state
    void reset_hidden_state(int batch_size = 1);

    // Set hidden state
    void set_hidden_state(const std::vector<float>& hxs);

    // Get hidden state
    std::vector<float> get_hidden_state() const;

    // Get observation size
    int get_obs_size() const { return obs_size_; }

    // Get action size (mean + logstd)
    int get_action_size() const { return action_size_; }

    // Simple JSON value storage (forward declaration - needs to be public for parser)
    struct JSONValue;

private:
    // Policy components
    std::vector<float> obs_mean_;
    std::vector<float> obs_var_;
    std::vector<EncoderLayer> encoder_layers_;
    GRUWeights gru_weights_;
    LinearLayer dist_linear_;

    // Hidden state: [num_layers, batch, hidden_size] - we use [1, 1, hidden_size]
    std::vector<float> hxs_;

    // Dimensions
    int obs_size_;
    int action_size_;
    int gru_hidden_size_;
    int gru_input_size_;

    // No-RNN path (use_rnn=False): when true, GRU is used; when false, encoder output goes directly to dist head
    bool has_rnn_;

    // When false, skip obs normalization (match rl_policyClean when checkpoint has no obs_normalizer)
    bool has_obs_normalizer_;

    // When non-empty, dist_linear outputs mean only; logstd comes from learned_stddev_ (adaptive_stddev=False)
    std::vector<float> learned_stddev_;

    // Swarm encoder (actor_encoder.*): when true, encoder_forward uses swarm_encoder_forward
    bool use_swarm_encoder_;
    std::vector<EncoderLayer> swarm_self_goal_layers_;
    std::vector<EncoderLayer> swarm_embedding_layers_;
    std::vector<EncoderLayer> swarm_value_layers_;
    std::vector<EncoderLayer> swarm_attention_layers_;  // last layer: linear only (no activation)

    // Constants
    static constexpr float EPS = 1e-5f;

    // Helper functions
    float activation_func(float x, Activation act) const;
    std::vector<float> linear_forward(const LinearLayer& layer, const std::vector<float>& input) const;
    std::vector<float> encoder_forward(const std::vector<float>& input) const;
    std::vector<float> swarm_encoder_forward(const std::vector<float>& input) const;
    std::vector<float> gru_forward(const std::vector<float>& input);  // Non-const because it modifies hxs_
    std::vector<float> normalize_obs(const std::vector<float>& obs) const;

    // JSON parsing helpers
    bool parse_json_file(const std::string& json_path);
    bool extract_tensor(const std::string& key, std::vector<float>& data, std::vector<int>& shape);
    void extract_flattened_array(std::shared_ptr<JSONValue> val, std::vector<float>& result);
    bool find_mlp_linear_indices(std::vector<int>& indices);
    bool has_key_in_weights(const std::string& key);
    bool is_swarm_encoder_json() const;
    bool build_encoder_from_json(Activation nonlinearity);
    bool build_swarm_encoder_from_json(Activation nonlinearity);
    bool load_gru_from_json();
    bool load_dist_linear_from_json();
    bool load_normalizer_from_json();

    // JSON data storage
    std::shared_ptr<JSONValue> json_data_;
};

#endif // RL_POLICY_JSON_H
